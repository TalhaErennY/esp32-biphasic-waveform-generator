#include <stdio.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

#include "driver/ledc.h"
#include "driver/gpio.h"
#include "driver/i2c.h"

#include "esp_err.h"
#include "esp_timer.h"
#include "esp_log.h"

#include "ssd1306.h"

#include "soc/gpio_struct.h"
#include "soc/gpio_reg.h"

// ---------------- GPIO ----------------
// PWM1 & PWM2 (rmt + ledc)
#define PWM1_GPIO   4
#define PWM2_GPIO   5

// BOOST pulse train output
#define BOOST_GPIO  7

// Buttons (low - active)
#define BTN_FREQ_GPIO     9
#define BTN_DUTY_GPIO     10
#define BTN_BOOST_UP_GPIO 12  
#define BTN_BOOST_DN_GPIO 11

// ---------------- OLED I2C (SDA=2, SCL=3) ----------------
#define OLED_I2C_PORT     I2C_NUM_0
#define OLED_ADDR         0x3C
#define OLED_SDA_GPIO     2
#define OLED_SCL_GPIO     3
#define OLED_I2C_FREQ_HZ  400000

static const ssd1306_i2c_t g_oled = {
    .port = OLED_I2C_PORT,
    .addr = OLED_ADDR,
    .sda_gpio = OLED_SDA_GPIO,
    .scl_gpio = OLED_SCL_GPIO,
    .freq_hz = OLED_I2C_FREQ_HZ
};

static int g_oled_ok = 0; // If OLED detected = 1

// ---------------- LEDC (PWM1/2) ----------------
#define PWM12_SPEED_MODE   LEDC_LOW_SPEED_MODE
#define PWM12_TIMER        LEDC_TIMER_0
#define PWM12_CH1          LEDC_CHANNEL_0
#define PWM12_CH2          LEDC_CHANNEL_1
#define PWM12_CLK          LEDC_USE_APB_CLK
#define PWM12_RES          LEDC_TIMER_10_BIT

static uint32_t s_max_duty12;
static const uint32_t s_gap_us = 20;

// ---- Frequency List (mHz) ----
static const uint32_t s_freq_mhz_list[] = {
    100,     // 0.1 Hz  (SW)
    500,     // 0.5 Hz  (SW)
    700,     // 0.7 Hz  (SW)
    1000,    // 1.0 Hz  (LEDC)
    1500,    // 1.5 Hz  (LEDC)
    2000,    // 2.0 Hz  (LEDC)
    50000,   // 50 Hz
    100000,  // 100 Hz
    200000,  // 200 Hz
    500000,  // 500 Hz
    1000000  // 1000 Hz
};
static const int s_freq_count = (int)(sizeof(s_freq_mhz_list) / sizeof(s_freq_mhz_list[0]));

// Duty presetleri (PWM1/2) (%)
static const float s_duty_list[] = { 0.1f, 0.5f, 1.0f, 2.0f, 5.0f, 10.0f, 20.0f };
static const int s_duty_count = (int)(sizeof(s_duty_list) / sizeof(s_duty_list[0]));

static int s_freq_idx = 7;  // default 100 Hz (100000 mHz)
static int s_duty_idx = 2;  // default 1.0%

// ---------------- BOOST pulse-train (esp_timer) ----------------
// TON=1us, TOFF=100us (TOFF changes with buttons)
static esp_timer_handle_t s_boost_timer = NULL;
static volatile int s_boost_active = 0;
static volatile uint32_t s_boost_ton_us  = 1;
static volatile uint32_t s_boost_toff_us = 100;
static volatile int s_boost_state_high = 0;

// ---------------- Core pinning ----------------
#if CONFIG_FREERTOS_UNICORE
  #define CORE_OLED 0
  #define CORE_PWM  0
#else
  #define CORE_OLED 0
  #define CORE_PWM  1
#endif

// ---------------- Concurrency guard ----------------
static portMUX_TYPE s_cfg_mux = portMUX_INITIALIZER_UNLOCKED;

// =====================================================
// Helpers
// =====================================================
static uint32_t pct_to_duty_ticks(float pct, uint32_t max_duty)
{
    if (pct < 0.0f) pct = 0.0f;
    if (pct > 100.0f) pct = 100.0f;
    return (uint32_t)((pct * (float)max_duty) / 100.0f);
}

static uint32_t mhz_to_hz_rounded(uint32_t mhz)
{
    // LEDC for only integer hz
    uint32_t hz = (mhz + 500) / 1000;
    if (hz == 0) hz = 1;
    return hz;
}

static uint32_t period_us_from_mhz_exact(uint32_t freq_mhz)
{
    if (freq_mhz == 0) return 0;
    return (uint32_t)(1000000000ULL / (uint64_t)freq_mhz); // exact mHz
}

static uint32_t us_to_ticks(uint32_t us, uint32_t period_us, uint32_t max_duty)
{
    if (period_us == 0) return 0;
    uint64_t num = (uint64_t)us * (uint64_t)max_duty + (period_us / 2);
    return (uint32_t)(num / period_us);
}

static void fmt_freq_mhz(char *out, size_t n, uint32_t mhz)
{
    uint32_t hz_i = mhz / 1000;
    uint32_t hz_f = mhz % 1000;

    if (mhz < 1000000) {
        snprintf(out, n, "%lu.%03luHz",
                 (unsigned long)hz_i, (unsigned long)hz_f);
    } else {
        uint32_t khz_i = mhz / 1000000;
        uint32_t khz_f = (mhz % 1000000) / 1000;
        snprintf(out, n, "%lu.%03lukHz",
                 (unsigned long)khz_i, (unsigned long)khz_f);
    }
}

// ---------------- I2C scan (OLED Check) ----------------
static int i2c_device_present(i2c_port_t port, uint8_t addr)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_stop(cmd);
    esp_err_t err = i2c_master_cmd_begin(port, cmd, pdMS_TO_TICKS(50));
    i2c_cmd_link_delete(cmd);
    return (err == ESP_OK);
}

// =====================================================
// BOOST pulse-train
// =====================================================
static inline void boost_gpio_high(void) { gpio_set_level(BOOST_GPIO, 1); } //normally 1
static inline void boost_gpio_low(void)  { gpio_set_level(BOOST_GPIO, 0); } //normally 0


static void boost_timer_arm_once(uint32_t us)
{
    if (us == 0) us = 1;
    esp_timer_start_once(s_boost_timer, us);
}

static void boost_pulsetrain_cb(void *arg)
{
    (void)arg;
    if (!s_boost_active) return;

    if (!s_boost_state_high) {
        boost_gpio_high();
        s_boost_state_high = 1;
        boost_timer_arm_once(s_boost_ton_us);
    } else {
        boost_gpio_low();
        s_boost_state_high = 0;
        boost_timer_arm_once(s_boost_toff_us);
    }
}

static void boost_pulsetrain_init(void)
{
    gpio_config_t io = {0};
    io.pin_bit_mask = (1ULL << BOOST_GPIO);
    io.mode = GPIO_MODE_OUTPUT;
    io.pull_up_en = GPIO_PULLUP_DISABLE;
    io.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io.intr_type = GPIO_INTR_DISABLE;
    ESP_ERROR_CHECK(gpio_config(&io));
    boost_gpio_low();

    esp_timer_create_args_t tcfg = {
        .callback = &boost_pulsetrain_cb,
        .arg = NULL,
        .dispatch_method = ESP_TIMER_TASK,
        .name = "boost_pulse"
    };
    ESP_ERROR_CHECK(esp_timer_create(&tcfg, &s_boost_timer));
}

static void boost_pulsetrain_start(uint32_t ton_us, uint32_t toff_us)
{
    if (ton_us == 0)  ton_us = 1;
    if (toff_us == 0) toff_us = 1;

    portENTER_CRITICAL(&s_cfg_mux);
    s_boost_ton_us  = ton_us;
    s_boost_toff_us = toff_us;
    portEXIT_CRITICAL(&s_cfg_mux);

    s_boost_state_high = 0;
    s_boost_active = 1;

    esp_timer_stop(s_boost_timer);
    boost_gpio_low();
    boost_timer_arm_once(1);

    float hz = 1000000.0f / (float)(ton_us + toff_us);
    printf("[BOOST] START TON=%luus TOFF=%luus (T=%luus ~%.1fHz)\n",
           (unsigned long)ton_us, (unsigned long)toff_us,
           (unsigned long)(ton_us + toff_us), hz);
}

static void boost_pulsetrain_stop(void)
{
    s_boost_active = 0;
    if (s_boost_timer) esp_timer_stop(s_boost_timer);
    boost_gpio_low();
    printf("[BOOST] STOP\n");
}

// =====================================================
// PWM12: LEDC part
// =====================================================
static esp_err_t pwm12_timer_set(uint32_t freq_hz)
{
    return ledc_timer_config(&(ledc_timer_config_t){
        .speed_mode       = PWM12_SPEED_MODE,
        .timer_num        = PWM12_TIMER,
        .duty_resolution  = PWM12_RES,
        .freq_hz          = freq_hz,
        .clk_cfg          = PWM12_CLK
    });
}

static void pwm12_ledc_init(uint32_t init_freq_hz)
{
    esp_err_t err = pwm12_timer_set(init_freq_hz);
    if (err != ESP_OK) {
        printf("[ERR] pwm12 timer init failed: %s\n", esp_err_to_name(err));
    }

    err = ledc_channel_config(&(ledc_channel_config_t){
        .gpio_num   = PWM1_GPIO,
        .speed_mode = PWM12_SPEED_MODE,
        .channel    = PWM12_CH1,
        .timer_sel  = PWM12_TIMER,
        .duty       = 0,
        .hpoint     = 0
    });
    if (err != ESP_OK) printf("[ERR] pwm1 channel failed: %s\n", esp_err_to_name(err));

    err = ledc_channel_config(&(ledc_channel_config_t){
        .gpio_num   = PWM2_GPIO,
        .speed_mode = PWM12_SPEED_MODE,
        .channel    = PWM12_CH2,
        .timer_sel  = PWM12_TIMER,
        .duty       = 0,
        .hpoint     = 0
    });
    if (err != ESP_OK) printf("[ERR] pwm2 channel failed: %s\n", esp_err_to_name(err));

    s_max_duty12 = (1U << PWM12_RES) - 1U;
}

static void pwm12_ledc_stop_and_gpio(void)
{
    // LEDC’yi durdur, pinleri GPIO output moduna al (SW PWM için)
    ledc_stop(PWM12_SPEED_MODE, PWM12_CH1, 0);
    ledc_stop(PWM12_SPEED_MODE, PWM12_CH2, 0);

    gpio_config_t io = {0};
    io.pin_bit_mask = (1ULL << PWM1_GPIO) | (1ULL << PWM2_GPIO);
    io.mode = GPIO_MODE_OUTPUT;
    io.pull_up_en = GPIO_PULLUP_DISABLE;
    io.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io.intr_type = GPIO_INTR_DISABLE;
    ESP_ERROR_CHECK(gpio_config(&io));

    gpio_set_level(PWM1_GPIO, 0);
    gpio_set_level(PWM2_GPIO, 0);
}

// LEDC ile uygula (>=1 Hz)
// PWM2 = PWM1 bittikten gap_us sonra başlar.
static void pwm12_ledc_apply(uint32_t freq_mhz, float duty_pct)
{
    uint32_t freq_hz = mhz_to_hz_rounded(freq_mhz);

    // LEDC period’ü, LEDC’ye verdiğin integer Hz’e göre tutarlı hesapla:
    uint32_t period_us = (uint32_t)(1000000UL / freq_hz);

    esp_err_t err = pwm12_timer_set(freq_hz);
    if (err != ESP_OK) {
        char ftxt[16];
        fmt_freq_mhz(ftxt, sizeof(ftxt), freq_mhz);
        printf("[ERR] ledc_timer_config failed for %s (hz=%lu): %s\n",
               ftxt, (unsigned long)freq_hz, esp_err_to_name(err));
        return;
    }

    uint32_t pwm1_pulse_us = (uint32_t)((float)period_us * duty_pct / 100.0f);
    uint32_t pwm2_delay_us = pwm1_pulse_us + s_gap_us;

    uint32_t duty_ticks  = pct_to_duty_ticks(duty_pct, s_max_duty12);
    uint32_t delay_ticks = us_to_ticks(pwm2_delay_us, period_us, s_max_duty12);

    if (delay_ticks + duty_ticks > s_max_duty12) {
        uint32_t old = duty_ticks;
        duty_ticks = (delay_ticks <= s_max_duty12) ? (s_max_duty12 - delay_ticks) : 0;
        printf("[WARN] PWM12 fit: duty_ticks %lu -> %lu\n",
               (unsigned long)old, (unsigned long)duty_ticks);
    }

    err = ledc_set_duty_with_hpoint(PWM12_SPEED_MODE, PWM12_CH1, duty_ticks, 0);
    if (err != ESP_OK) { printf("[ERR] set pwm1 duty failed: %s\n", esp_err_to_name(err)); return; }
    err = ledc_update_duty(PWM12_SPEED_MODE, PWM12_CH1);
    if (err != ESP_OK) { printf("[ERR] upd pwm1 duty failed: %s\n", esp_err_to_name(err)); return; }

    err = ledc_set_duty_with_hpoint(PWM12_SPEED_MODE, PWM12_CH2, duty_ticks, delay_ticks);
    if (err != ESP_OK) { printf("[ERR] set pwm2 duty failed: %s\n", esp_err_to_name(err)); return; }
    err = ledc_update_duty(PWM12_SPEED_MODE, PWM12_CH2);
    if (err != ESP_OK) { printf("[ERR] upd pwm2 duty failed: %s\n", esp_err_to_name(err)); return; }

    uint32_t f_rb = ledc_get_freq(PWM12_SPEED_MODE, PWM12_TIMER);
    uint32_t pw_us = (uint32_t)((uint64_t)duty_ticks * period_us / s_max_duty12);

    char ftxt[16];
    fmt_freq_mhz(ftxt, sizeof(ftxt), freq_mhz);

    printf("[PWM12-LEDC] idx=%d/%d freq_req=%s -> set=%luHz rb=%luHz duty=%.2f%% (~%luus) PWM2_start=%luus gap=%luus\n",
           s_freq_idx, s_freq_count-1,
           ftxt,
           (unsigned long)freq_hz, (unsigned long)f_rb,
           duty_pct, (unsigned long)pw_us,
           (unsigned long)pwm2_delay_us, (unsigned long)s_gap_us);
}

// =====================================================
// PWM12: Software (esp_timer) for <1Hz
// =====================================================
typedef enum {
    SWPHASE_START = 0,
    SWPHASE_PWM2_ON,
    SWPHASE_PWM1_OFF,
    SWPHASE_PWM2_OFF,
    SWPHASE_PERIOD_END
} sw_phase_t;

static esp_timer_handle_t s_sw_timer = NULL;
static volatile int s_sw_active = 0;

static volatile uint32_t s_sw_period_us = 0;
static volatile uint32_t s_sw_duty_us   = 0;
static volatile uint32_t s_sw_pwm2_delay_us = 0;

static volatile sw_phase_t s_sw_phase = SWPHASE_START;

static void sw_timer_arm_once(uint32_t us)
{
    if (us == 0) us = 1;
    esp_timer_start_once(s_sw_timer, us);
}

static void sw_pwm12_cb(void *arg)
{
    (void)arg;
    if (!s_sw_active) return;

    switch (s_sw_phase) {
        case SWPHASE_START:
            gpio_set_level(PWM1_GPIO, 1);
            gpio_set_level(PWM2_GPIO, 0);
            s_sw_phase = SWPHASE_PWM2_ON;
            sw_timer_arm_once(s_sw_pwm2_delay_us);
            break;

        case SWPHASE_PWM2_ON:
            gpio_set_level(PWM2_GPIO, 1);
            s_sw_phase = SWPHASE_PWM1_OFF;
            sw_timer_arm_once(s_sw_duty_us);
            break;

        case SWPHASE_PWM1_OFF:
            gpio_set_level(PWM1_GPIO, 0);
            s_sw_phase = SWPHASE_PWM2_OFF;
            sw_timer_arm_once(s_sw_duty_us);
            break;

        case SWPHASE_PWM2_OFF:
            gpio_set_level(PWM2_GPIO, 0);
            {
                uint64_t used = (uint64_t)s_sw_pwm2_delay_us + (uint64_t)s_sw_duty_us + (uint64_t)s_sw_duty_us;
                uint32_t rest = (used < s_sw_period_us) ? (uint32_t)(s_sw_period_us - used) : 1;
                s_sw_phase = SWPHASE_PERIOD_END;
                sw_timer_arm_once(rest);
            }
            break;

        case SWPHASE_PERIOD_END:
        default:
            s_sw_phase = SWPHASE_START;
            sw_timer_arm_once(1);
            break;
    }
}

static void pwm12_sw_start(uint32_t freq_mhz, float duty_pct)
{
    pwm12_ledc_stop_and_gpio();

    uint32_t period_us = period_us_from_mhz_exact(freq_mhz);
    if (period_us < 1000) period_us = 1000;

    uint32_t duty_us = (uint32_t)((float)period_us * duty_pct / 100.0f);
    uint32_t pwm2_delay_us = duty_us + s_gap_us;

    uint64_t used = (uint64_t)pwm2_delay_us + (uint64_t)duty_us + (uint64_t)duty_us;
    if (used >= period_us) {
        uint32_t max_duty_us = (period_us > pwm2_delay_us) ? (uint32_t)((period_us - pwm2_delay_us) / 2) : 0;
        duty_us = max_duty_us;
        printf("[WARN] SW PWM12 fit: duty_us adjusted to %luus\n", (unsigned long)duty_us);
        pwm2_delay_us = duty_us + s_gap_us;
    }

    s_sw_period_us = period_us;
    s_sw_duty_us = duty_us;
    s_sw_pwm2_delay_us = pwm2_delay_us;

    s_sw_phase = SWPHASE_START;
    s_sw_active = 1;

    char ftxt[16];
    fmt_freq_mhz(ftxt, sizeof(ftxt), freq_mhz);

    printf("[PWM12-SW] idx=%d/%d freq=%s period=%luus duty=%.2f%% duty_us=%lu PWM2_start=%luus gap=%luus\n",
           s_freq_idx, s_freq_count-1,
           ftxt,
           (unsigned long)period_us,
           duty_pct,
           (unsigned long)duty_us,
           (unsigned long)pwm2_delay_us,
           (unsigned long)s_gap_us);

    esp_timer_stop(s_sw_timer);
    sw_timer_arm_once(1);
}

static void pwm12_sw_stop(void)
{
    s_sw_active = 0;
    if (s_sw_timer) esp_timer_stop(s_sw_timer);
    gpio_set_level(PWM1_GPIO, 0);
    gpio_set_level(PWM2_GPIO, 0);
}

static void pwm12_apply(uint32_t freq_mhz, float duty_pct)
{
    if (freq_mhz < 1000) {
        pwm12_sw_start(freq_mhz, duty_pct);
    } else {
        if (s_sw_active) pwm12_sw_stop();
        pwm12_ledc_apply(freq_mhz, duty_pct);
    }
}

// =====================================================
// Buttons
// =====================================================
static void buttons_init(void)
{
    gpio_config_t io = {0};
    io.pin_bit_mask =
        (1ULL << BTN_FREQ_GPIO) |
        (1ULL << BTN_DUTY_GPIO) |
        (1ULL << BTN_BOOST_UP_GPIO) |
        (1ULL << BTN_BOOST_DN_GPIO);
    io.mode = GPIO_MODE_INPUT;
    io.pull_up_en = GPIO_PULLUP_ENABLE;
    io.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io.intr_type = GPIO_INTR_DISABLE;
    ESP_ERROR_CHECK(gpio_config(&io));
}

static inline int btn_pressed(gpio_num_t pin)
{
    return gpio_get_level(pin) == 0;
}

// =====================================================
// OLED UI (no flicker: update only on change)
// =====================================================
static void oled_show_welcome(void)
{
    if (!g_oled_ok) return;
    ssd1306_clear(&g_oled);
    ssd1306_draw_text_8x8(&g_oled, 0, 1, "Baslatiliyor...");
    ssd1306_draw_text_8x8(&g_oled, 0, 3, "Tensprot v1.1");
    ssd1306_draw_text_8x8(&g_oled, 0, 5, "Powered by TEY");
}

static void oled_render_config_snapshot(int freq_idx, int duty_idx)
{
    if (!g_oled_ok) return;

    char l0[32], l1[32], l2[32], l3[32], l4[32], l5[32];
    char f12txt[16];

    uint32_t f12_mhz = s_freq_mhz_list[freq_idx];
    fmt_freq_mhz(f12txt, sizeof(f12txt), f12_mhz);

    float duty12 = s_duty_list[duty_idx];

    uint32_t ton, toff;
    portENTER_CRITICAL(&s_cfg_mux);
    ton  = s_boost_ton_us;
    toff = s_boost_toff_us;
    portEXIT_CRITICAL(&s_cfg_mux);

    snprintf(l0, sizeof(l0), "PWM CONFIG");
    snprintf(l1, sizeof(l1), "PWM12 F:%s", f12txt);
    snprintf(l2, sizeof(l2), "PWM12 D:%.1f%%", duty12);
    snprintf(l3, sizeof(l3), "PWM2 GAP:%luus", (unsigned long)s_gap_us);
    snprintf(l4, sizeof(l4), "BOOST TON:%luus", (unsigned long)ton);
    snprintf(l5, sizeof(l5), "BOOST TOFF:%luus", (unsigned long)toff);

    ssd1306_clear(&g_oled);
    ssd1306_draw_text_8x8(&g_oled, 0, 0, l0);
    ssd1306_draw_text_8x8(&g_oled, 0, 2, l1);
    ssd1306_draw_text_8x8(&g_oled, 0, 3, l2);
    ssd1306_draw_text_8x8(&g_oled, 0, 4, l3);
    ssd1306_draw_text_8x8(&g_oled, 0, 6, l4);
    ssd1306_draw_text_8x8(&g_oled, 0, 7, l5);
}

static void oled_task(void *arg)
{
    (void)arg;

    int last_freq = -1;
    int last_duty = -1;
    uint32_t last_ton = 0xFFFFFFFF;
    uint32_t last_toff = 0xFFFFFFFF;

    while (1) {
        if (!g_oled_ok) {
            vTaskDelay(pdMS_TO_TICKS(500));
            continue;
        }

        int f, d;
        uint32_t ton, toff;

        portENTER_CRITICAL(&s_cfg_mux);
        f = s_freq_idx;
        d = s_duty_idx;
        ton  = s_boost_ton_us;
        toff = s_boost_toff_us;
        portEXIT_CRITICAL(&s_cfg_mux);

        if (f != last_freq || d != last_duty || ton != last_ton || toff != last_toff) {
            last_freq = f;
            last_duty = d;
            last_ton  = ton;
            last_toff = toff;
            oled_render_config_snapshot(f, d);
        }

        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

// =====================================================
// Control task (PWM core)
// =====================================================
static void control_task(void *arg)
{
    (void)arg;

    int last_f=0, last_d=0, last_u=0, last_n=0;
    int st_f=0,   st_d=0,   st_u=0,   st_n=0;
    int cnt_f=0,  cnt_d=0,  cnt_u=0,  cnt_n=0;

    const int debounce_ticks = 3;

    // ilk apply
    uint32_t fmhz;
    float dpct;

    portENTER_CRITICAL(&s_cfg_mux);
    fmhz = s_freq_mhz_list[s_freq_idx];
    dpct = s_duty_list[s_duty_idx];
    portEXIT_CRITICAL(&s_cfg_mux);

    pwm12_apply(fmhz, dpct);

    while (1) {
        int now_f = btn_pressed(BTN_FREQ_GPIO);
        int now_d = btn_pressed(BTN_DUTY_GPIO);
        int now_u = btn_pressed(BTN_BOOST_UP_GPIO);
        int now_n = btn_pressed(BTN_BOOST_DN_GPIO);

        // ---- FREQ ----
        if (now_f == last_f) { if (cnt_f < debounce_ticks) cnt_f++; }
        else { cnt_f=0; last_f=now_f; }
        if (cnt_f == debounce_ticks && st_f != now_f) {
            st_f = now_f;
            if (st_f) {
                portENTER_CRITICAL(&s_cfg_mux);
                int next = s_freq_idx + 1;
                if (next >= s_freq_count) next = 0;
                s_freq_idx = next;
                fmhz = s_freq_mhz_list[s_freq_idx];
                dpct = s_duty_list[s_duty_idx];
                portEXIT_CRITICAL(&s_cfg_mux);

                printf("[BTN] FREQ -> idx=%d\n", s_freq_idx);
                pwm12_apply(fmhz, dpct);
            }
        }

        // ---- DUTY ----
        if (now_d == last_d) { if (cnt_d < debounce_ticks) cnt_d++; }
        else { cnt_d=0; last_d=now_d; }
        if (cnt_d == debounce_ticks && st_d != now_d) {
            st_d = now_d;
            if (st_d) {
                portENTER_CRITICAL(&s_cfg_mux);
                int next = s_duty_idx + 1;
                if (next >= s_duty_count) next = 0;
                s_duty_idx = next;
                fmhz = s_freq_mhz_list[s_freq_idx];
                dpct = s_duty_list[s_duty_idx];
                portEXIT_CRITICAL(&s_cfg_mux);

                printf("[BTN] DUTY -> idx=%d\n", s_duty_idx);
                pwm12_apply(fmhz, dpct);
            }
        }

        // ---- BOOST TOFF UP (+10us) ----
        if (now_u == last_u) { if (cnt_u < debounce_ticks) cnt_u++; }
        else { cnt_u=0; last_u=now_u; }
        if (cnt_u == debounce_ticks && st_u != now_u) {
            st_u = now_u;
            if (st_u) {
                uint32_t ton, toff;
                portENTER_CRITICAL(&s_cfg_mux);
                if (s_boost_toff_us < 2000) s_boost_toff_us += 10;
                ton  = s_boost_ton_us;
                toff = s_boost_toff_us;
                portEXIT_CRITICAL(&s_cfg_mux);
                boost_pulsetrain_start(ton, toff);
            }
        }

        // ---- BOOST TOFF DOWN (-10us) ----
        if (now_n == last_n) { if (cnt_n < debounce_ticks) cnt_n++; }
        else { cnt_n=0; last_n=now_n; }
        if (cnt_n == debounce_ticks && st_n != now_n) {
            st_n = now_n;
            if (st_n) {
                uint32_t ton, toff;
                portENTER_CRITICAL(&s_cfg_mux);
                if (s_boost_toff_us > 10) s_boost_toff_us -= 10;
                ton  = s_boost_ton_us;
                toff = s_boost_toff_us;
                portEXIT_CRITICAL(&s_cfg_mux);
                boost_pulsetrain_start(ton, toff);
            }
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

// =====================================================
// app_main
// =====================================================
void app_main(void)
{
    printf("app_main entered\n");

    // esp_timer: SW PWM12 için
    {
        esp_timer_create_args_t tcfg = {
            .callback = &sw_pwm12_cb,
            .arg = NULL,
            .dispatch_method = ESP_TIMER_TASK,
            .name = "sw_pwm12"
        };
        ESP_ERROR_CHECK(esp_timer_create(&tcfg, &s_sw_timer));
    }

    // I2C init
    ssd1306_i2c_init(&g_oled);

    // OLED var mı kontrol et
    g_oled_ok = i2c_device_present(g_oled.port, g_oled.addr);
    printf("OLED probe addr 0x%02X: %s\n", g_oled.addr, g_oled_ok ? "OK" : "NOT FOUND");

    if (g_oled_ok) {
        ssd1306_init(&g_oled);
        oled_show_welcome();
        vTaskDelay(pdMS_TO_TICKS(900));
    } else {
        printf("[WARN] OLED not found. Running without display.\n");
    }

    // PWM12 init
    uint32_t init_mhz = s_freq_mhz_list[s_freq_idx];
    uint32_t init_hz  = mhz_to_hz_rounded(init_mhz);
    pwm12_ledc_init(init_hz);

    // BOOST pulse-train init + start (Arduino gibi)
    boost_pulsetrain_init();
    boost_pulsetrain_start(s_boost_ton_us, s_boost_toff_us);

    // Buttons + tasks
    buttons_init();

    // Control task (PWM core)
    xTaskCreatePinnedToCore(control_task, "control_task", 4096, NULL, 5, NULL, CORE_PWM);

    // OLED task (OLED core)
    if (g_oled_ok) {
        xTaskCreatePinnedToCore(oled_task, "oled_task", 4096, NULL, 4, NULL, CORE_OLED);
    }

    printf("System started. OLED SDA=%d SCL=%d, OLED_CORE=%d PWM_CORE=%d\n",
           OLED_SDA_GPIO, OLED_SCL_GPIO, CORE_OLED, CORE_PWM);
}
