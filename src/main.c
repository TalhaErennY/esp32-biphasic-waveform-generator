#include <stdio.h>
#include <math.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "driver/gpio.h"
#include "esp_err.h"

#include "app_config.h"
#include "app_state.h"

#include "buttons.h"
#include "pwm12.h"
#include "boost_pwm.h"
#include "adc_current.h"
#include "current_ctrl.h"
#include "ui_oled.h"

#if CONFIG_FREERTOS_UNICORE
  #define CORE_UI   0
  #define CORE_BTN  0
#else
  #define CORE_UI   0
  #define CORE_BTN  0
#endif

static float clampf(float x, float a, float b)
{
    if (x < a) return a;
    if (x > b) return b;
    return x;
}

static void apply_pwm12_from_state(void)
{
    uint32_t f, p;
    portENTER_CRITICAL(&g_app.mux);
    f = g_pwm12_freq_hz_list[g_app.pwm12_freq_idx];
    p = g_pwm12_pulse_us_list[g_app.pwm12_pulse_idx];
    portEXIT_CRITICAL(&g_app.mux);

    pwm12_set(f, p);
}

/**
 * CAL GPIO'yu buttons.c'ye dokunmadan input+pullup yapıyoruz.
 * Çünkü buttons_init sadece struct'taki 4 pini config ediyor.
 */
static void cal_button_gpio_init(void)
{
    gpio_config_t io = {0};
    io.mode = GPIO_MODE_INPUT;
    io.pull_up_en = GPIO_PULLUP_ENABLE;
    io.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io.intr_type = GPIO_INTR_DISABLE;
    io.pin_bit_mask = (1ULL << BTN_CAL_GPIO);
    ESP_ERROR_CHECK(gpio_config(&io));
}

static void button_task(void *arg)
{
    (void)arg;

    const int debounce_ticks = 3;

    int last_f=0, st_f=0, cnt_f=0;
    int last_p=0, st_p=0, cnt_p=0;
    int last_u=0, st_u=0, cnt_u=0;
    int last_d=0, st_d=0, cnt_d=0;
    int last_c=0, st_c=0, cnt_c=0;

    while (1) {
        int now_f = button_pressed(BTN_FREQ_GPIO);
        int now_p = button_pressed(BTN_PULSE_GPIO);
        int now_u = button_pressed(BTN_I_UP_GPIO);
        int now_d = button_pressed(BTN_I_DN_GPIO);
        int now_c = button_pressed(BTN_CAL_GPIO);

        // ---- FREQ ----
        if (now_f == last_f) { if (cnt_f < debounce_ticks) cnt_f++; }
        else { cnt_f=0; last_f=now_f; }
        if (cnt_f == debounce_ticks && st_f != now_f) {
            st_f = now_f;
            if (st_f) {
                portENTER_CRITICAL(&g_app.mux);
                g_app.pwm12_freq_idx++;
                if (g_app.pwm12_freq_idx >= g_pwm12_freq_count) g_app.pwm12_freq_idx = 0;
                portEXIT_CRITICAL(&g_app.mux);
                apply_pwm12_from_state();
            }
        }

        // ---- PULSE(us) ----
        if (now_p == last_p) { if (cnt_p < debounce_ticks) cnt_p++; }
        else { cnt_p=0; last_p=now_p; }
        if (cnt_p == debounce_ticks && st_p != now_p) {
            st_p = now_p;
            if (st_p) {
                portENTER_CRITICAL(&g_app.mux);
                g_app.pwm12_pulse_idx++;
                if (g_app.pwm12_pulse_idx >= g_pwm12_pulse_count) g_app.pwm12_pulse_idx = 0;
                portEXIT_CRITICAL(&g_app.mux);
                apply_pwm12_from_state();
            }
        }

        // ---- I SET +0.5mA ----
        if (now_u == last_u) { if (cnt_u < debounce_ticks) cnt_u++; }
        else { cnt_u=0; last_u=now_u; }
        if (cnt_u == debounce_ticks && st_u != now_u) {
            st_u = now_u;
            if (st_u) {
                portENTER_CRITICAL(&g_app.mux);
                g_app.current_set_ma = clampf(g_app.current_set_ma + CURRENT_STEP_MA,
                                              CURRENT_MIN_MA, CURRENT_MAX_MA);
                portEXIT_CRITICAL(&g_app.mux);
                printf("[BTN] I_set = %.2f mA\n", (double)g_app.current_set_ma);
            }
        }

        // ---- I SET -0.5mA ----
        if (now_d == last_d) { if (cnt_d < debounce_ticks) cnt_d++; }
        else { cnt_d=0; last_d=now_d; }
        if (cnt_d == debounce_ticks && st_d != now_d) {
            st_d = now_d;
            if (st_d) {
                portENTER_CRITICAL(&g_app.mux);
                g_app.current_set_ma = clampf(g_app.current_set_ma - CURRENT_STEP_MA,
                                              CURRENT_MIN_MA, CURRENT_MAX_MA);
                portEXIT_CRITICAL(&g_app.mux);
                printf("[BTN] I_set = %.2f mA\n", (double)g_app.current_set_ma);
            }
        }

        // ---- CAL: k = I_set / area_filt (RAM) ----
        if (now_c == last_c) { if (cnt_c < debounce_ticks) cnt_c++; }
        else { cnt_c=0; last_c=now_c; }
        if (cnt_c == debounce_ticks && st_c != now_c) {
            st_c = now_c;
            if (st_c) {
                float set_ma, area_f;
                portENTER_CRITICAL(&g_app.mux);
                set_ma = g_app.current_set_ma;
                area_f = g_app.area_filt;
                portEXIT_CRITICAL(&g_app.mux);

                if (area_f > 1.0f) {
                    float newk = set_ma / area_f;

                    portENTER_CRITICAL(&g_app.mux);
                    // yumuşak güncelleme
                    g_app.k_area_to_ma = 0.85f * g_app.k_area_to_ma + 0.15f * newk;
                    portEXIT_CRITICAL(&g_app.mux);

                    printf("[CAL] k updated -> %.6f (new=%.6f)\n",
                           (double)g_app.k_area_to_ma, (double)newk);
                } else {
                    printf("[CAL] ignored (area too small)\n");
                }
            }
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void app_main(void)
{
    printf("app_main entered\n");

    // OLED
    ui_oled_init();
    if (ui_oled_is_ok()) {
        ui_oled_show_welcome();
        vTaskDelay(pdMS_TO_TICKS(700));
    } else {
        printf("[WARN] OLED not found.\n");
    }

    // BOOST
    boost_pwm_init(BOOST_GPIO);
    boost_pwm_set_duty_pct(0);

    // ADC
    adc_current_init();

    // PWM12
    pwm12_init(PWM1_GPIO, PWM2_GPIO, PWM12_GAP_US);

    // PWM12 -> ADC senkron callback
    pwm12_set_pulse_callback(current_ctrl_notify_pulse_window, NULL);

    // state'ten ilk PWM12 uygula
    apply_pwm12_from_state();

    // current control task
    current_ctrl_init();

    // Buttons (ESKİ çalışan API ile uyumlu!)
    buttons_init((buttons_pins_t){
        .freq  = BTN_FREQ_GPIO,
        .pulse = BTN_PULSE_GPIO,
        .up    = BTN_I_UP_GPIO,
        .dn    = BTN_I_DN_GPIO,
    });

    // CAL GPIO'yu ayrıca init et
    cal_button_gpio_init();

    // Button task
    xTaskCreatePinnedToCore(button_task, "button_task", 4096, NULL, 7, NULL, CORE_BTN);

    // OLED task
    if (ui_oled_is_ok()) {
        xTaskCreatePinnedToCore(ui_oled_task, "oled_task", 4096, NULL, 4, NULL, CORE_UI);
    }

    printf("System started.\n");
}
