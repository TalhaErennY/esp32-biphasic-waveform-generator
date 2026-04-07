#include "pwm12.h"

#include <stdio.h>
#include "driver/gpio.h"
#include "esp_timer.h"
#include "esp_err.h"
#include "esp_rom_sys.h"

static int s_gpio1 = -1;
static int s_gpio2 = -1;
static uint32_t s_gap_us = 20;

static esp_timer_handle_t s_tmr = NULL;

static volatile uint32_t s_freq_hz = 100;
static volatile uint32_t s_pulse_us = 100;

static pwm12_pulse_cb_t s_cb = NULL;
static void *s_cb_user = NULL;

static uint32_t compute_window_us(uint32_t pulse_us)
{
    // iki pulse + gap + extra
    return (2U * pulse_us) + s_gap_us + 80;
}

void pwm12_set_pulse_callback(pwm12_pulse_cb_t cb, void *user)
{
    s_cb = cb;
    s_cb_user = user;
}

static void pwm12_timer_cb(void *arg)
{
    (void)arg;

    // PWM1 ON
    gpio_set_level(s_gpio1, 1);
    gpio_set_level(s_gpio2, 0);

    // ADC ölçüm penceresini başlat
    if (s_cb) {
        uint32_t win_us = compute_window_us(s_pulse_us);
        s_cb(win_us, s_cb_user);
    }

    // PWM1 OFF after pulse
    esp_rom_delay_us(s_pulse_us);
    gpio_set_level(s_gpio1, 0);

    // gap
    esp_rom_delay_us(s_gap_us);

    // PWM2 ON
    gpio_set_level(s_gpio2, 1);
    esp_rom_delay_us(s_pulse_us);
    gpio_set_level(s_gpio2, 0);
}

void pwm12_init(int gpio1, int gpio2, uint32_t gap_us)
{
    s_gpio1 = gpio1;
    s_gpio2 = gpio2;
    s_gap_us = gap_us;

    gpio_config_t io = {0};
    io.pin_bit_mask = (1ULL << s_gpio1) | (1ULL << s_gpio2);
    io.mode = GPIO_MODE_OUTPUT;
    io.pull_up_en = 0;
    io.pull_down_en = 0;
    io.intr_type = GPIO_INTR_DISABLE;
    ESP_ERROR_CHECK(gpio_config(&io));

    gpio_set_level(s_gpio1, 0);
    gpio_set_level(s_gpio2, 0);

    esp_timer_create_args_t tcfg = {
        .callback = &pwm12_timer_cb,
        .arg = NULL,
        .dispatch_method = ESP_TIMER_TASK,
        .name = "pwm12"
    };
    ESP_ERROR_CHECK(esp_timer_create(&tcfg, &s_tmr));
}

void pwm12_set(uint32_t freq_hz, uint32_t pulse_us)
{
    if (freq_hz < 1) freq_hz = 1;
    if (pulse_us < 10) pulse_us = 10;

    s_freq_hz = freq_hz;
    s_pulse_us = pulse_us;

    uint64_t period_us = 1000000ULL / (uint64_t)s_freq_hz;

    // güvenlik: periyot içine sığmazsa kıs
    uint64_t used = (uint64_t)(2U * s_pulse_us) + (uint64_t)s_gap_us + 20ULL;
    if (used >= period_us) {
        // pulse'u periyoda sığacak kadar küçült
        uint64_t max_pulse = (period_us > (s_gap_us + 40ULL)) ? ((period_us - (s_gap_us + 40ULL)) / 2ULL) : 10ULL;
        if (max_pulse < 10) max_pulse = 10;
        s_pulse_us = (uint32_t)max_pulse;
        printf("[PWM12] WARN: pulse trimmed to %lu us to fit period\n", (unsigned long)s_pulse_us);
    }

    // timer restart (ESP_ERR_INVALID_STATE -> ignore)
    if (s_tmr) {
        esp_err_t e = esp_timer_stop(s_tmr);
        if (e != ESP_OK && e != ESP_ERR_INVALID_STATE) {
            ESP_ERROR_CHECK(e);
        }
        ESP_ERROR_CHECK(esp_timer_start_periodic(s_tmr, (uint32_t)period_us));
    }

    printf("[PWM12] set f=%lu Hz pulse=%lu us gap=%lu us\n",
           (unsigned long)s_freq_hz, (unsigned long)s_pulse_us, (unsigned long)s_gap_us);
}
