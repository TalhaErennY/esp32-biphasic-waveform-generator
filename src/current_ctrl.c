#include "current_ctrl.h"

#include <stdio.h>
#include <math.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "app_config.h"
#include "app_state.h"
#include "boost_pwm.h"
#include "adc_current.h"

static float clampf(float x, float a, float b)
{
    if (x < a) return a;
    if (x > b) return b;
    return x;
}

static void control_task(void *arg)
{
    (void)arg;

    TickType_t last = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(5); // 200 Hz -> 5ms
    const TickType_t period_safe = (period == 0) ? 1 : period;

    // dt (saniye) — integratör için şart
    const float dt = 0.005f;

    while (1) {
        // state oku
        float set_ma, area_filt, k, meas_ma, err;

        portENTER_CRITICAL(&g_app.mux);
        set_ma    = g_app.current_set_ma;
        area_filt = g_app.area_filt;
        k         = g_app.k_area_to_ma;
        portEXIT_CRITICAL(&g_app.mux);

        meas_ma = k * area_filt;
        err = set_ma - meas_ma;

        int duty_to_set;

        portENTER_CRITICAL(&g_app.mux);

        // ----- Anti-windup + dt'li PI -----
        // Önce sadece P ile tahmini duty (satür kontrolü için)
        float u_p = PI_KP * err;
        float d_pre = (float)g_app.boost_duty_pct + u_p;

        int at_max = (g_app.boost_duty_pct >= BOOST_DUTY_MAX_PCT);
        int at_min = (g_app.boost_duty_pct <= BOOST_DUTY_MIN_PCT);

        // Satürde değilse integratöre izin ver.
        // Satürdeyse sadece "satürden çıkmaya yarayacak yönde" integratöre izin ver.
        int allow_i =
            (!at_max && !at_min) ||
            (at_max && err < 0) ||   // max'ta ama hata azalt diyor
            (at_min && err > 0);     // min'de ama hata artır diyor

        if (allow_i) {
            g_app.i_err += err * dt;
            g_app.i_err = clampf(g_app.i_err, -PI_I_CLAMP, PI_I_CLAMP);
        }

        float i_err = g_app.i_err;

        float u = (PI_KP * err) + (PI_KI * i_err);

        float d_new = (float)g_app.boost_duty_pct + u;
        d_new = clampf(d_new, (float)BOOST_DUTY_MIN_PCT, (float)BOOST_DUTY_MAX_PCT);

        g_app.boost_duty_pct = (int)lroundf(d_new);
        duty_to_set = g_app.boost_duty_pct;

        // UI/debug için ölçümü yaz
        g_app.current_meas_ma = meas_ma;

        portEXIT_CRITICAL(&g_app.mux);

        boost_pwm_set_duty_pct(duty_to_set);

        vTaskDelayUntil(&last, period_safe);
    }
}

// PWM12 pulse başladığında çağrılacak callback:
// ADC’ye “şu pencereyi ölç” diye sinyal gönderir.
void current_ctrl_notify_pulse_window(uint32_t win_us, void *user)
{
    (void)user;
    adc_current_request_capture(win_us);
}

void current_ctrl_init(void)
{
    // kontrol task CORE1’de
    xTaskCreatePinnedToCore(control_task, "ctrl_task", 4096, NULL, 5, NULL, 1);
}
