#include "adc_current.h"

#include <stdio.h>
#include <math.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "esp_err.h"

#include "app_config.h"
#include "app_state.h"

typedef struct {
    uint32_t win_us;
} cap_req_t;

static QueueHandle_t s_q = NULL;

static adc_oneshot_unit_handle_t s_adc = NULL;
static adc_cali_handle_t s_cali = NULL;

static float iir_update(float y, float x, float a)
{
    // y += a*(x-y)
    return y + a * (x - y);
}

// "alan" metriği: pencere boyunca ADC örneklerinin (volt - baseline) toplamı gibi
static float capture_area(uint32_t win_us)
{
    // Basit yaklaşım: pencere boyunca hızlıca N örnek al
    // (senin mevcut kodun buna göre yazılmıştı)
    const int sample_delay_us = 50; // örnekler arası
    int n = (int)(win_us / sample_delay_us);
    if (n < 1) n = 1;
    if (n > 2000) n = 2000;

    float area = 0.0f;

    for (int i = 0; i < n; i++) {
        int raw = 0;
        int mv  = 0;

        esp_err_t e = adc_oneshot_read(s_adc, ADC_CHANNEL_USED, &raw);
        if (e == ESP_OK) {
            if (s_cali) {
                (void)adc_cali_raw_to_voltage(s_cali, raw, &mv);
                // baseline çıkarma vs. (senin projendeki mantığa göre)
                // Şimdilik direkt mv topluyoruz: "alan"
                area += (float)mv;
            } else {
                area += (float)raw;
            }
        }

        // çok kısa bekleme
        ets_delay_us(sample_delay_us);
    }

    return area;
}

static void adc_task(void *arg)
{
    (void)arg;

    while (1) {
        cap_req_t req;
        if (xQueueReceive(s_q, &req, portMAX_DELAY) != pdTRUE) continue;

        float area = capture_area(req.win_us);

        portENTER_CRITICAL(&g_app.mux);
        g_app.area = area;
        g_app.area_filt = iir_update(g_app.area_filt, area, AREA_IIR_ALPHA);
        portEXIT_CRITICAL(&g_app.mux);
    }
}

void adc_current_init(void)
{
    // ADC oneshot init
    adc_oneshot_unit_init_cfg_t init_cfg = {
        .unit_id = ADC_UNIT_USED,
        .ulp_mode = ADC_ULP_MODE_DISABLE
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_cfg, &s_adc));

    adc_oneshot_chan_cfg_t chan_cfg = {
        .atten = ADC_ATTEN_USED,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(s_adc, ADC_CHANNEL_USED, &chan_cfg));

    // calibration (varsa)
    adc_cali_handle_t cali = NULL;
    bool cali_ok = false;

#if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
    {
        adc_cali_curve_fitting_config_t cali_cfg = {
            .unit_id = ADC_UNIT_USED,
            .chan = ADC_CHANNEL_USED,
            .atten = ADC_ATTEN_USED,
            .bitwidth = ADC_BITWIDTH_DEFAULT,
        };
        if (adc_cali_create_scheme_curve_fitting(&cali_cfg, &cali) == ESP_OK) {
            cali_ok = true;
        }
    }
#elif ADC_CALI_SCHEME_LINE_FITTING_SUPPORTED
    {
        adc_cali_line_fitting_config_t cali_cfg = {
            .unit_id = ADC_UNIT_USED,
            .atten = ADC_ATTEN_USED,
            .bitwidth = ADC_BITWIDTH_DEFAULT,
        };
        if (adc_cali_create_scheme_line_fitting(&cali_cfg, &cali) == ESP_OK) {
            cali_ok = true;
        }
    }
#endif

    if (cali_ok) {
        s_cali = cali;
    } else {
        s_cali = NULL;
    }

    // ÖNEMLİ: 1 elemanlı queue -> her zaman en güncel capture isteği kalsın
    s_q = xQueueCreate(1, sizeof(cap_req_t));
    if (!s_q) {
        printf("adc_current: queue create failed\n");
        return;
    }

    // ADC task CORE1’de
    xTaskCreatePinnedToCore(adc_task, "adc_task", 4096, NULL, 6, NULL, 1);
}

void adc_current_request_capture(uint32_t window_us)
{
    // pencereyi pulse’a göre aç
    cap_req_t req = { .win_us = window_us + ADC_WIN_EXTRA_US };

    // her zaman en güncel pencere isteği kalsın (1 elemanlı queue + overwrite)
    (void)xQueueOverwrite(s_q, &req);
}
