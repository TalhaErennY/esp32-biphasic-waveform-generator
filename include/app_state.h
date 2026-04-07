#pragma once
#include "freertos/FreeRTOS.h"
#include "freertos/portmacro.h"
#include <stdint.h>

typedef struct {
    // PWM12 seçimleri
    int pwm12_freq_idx;
    int pwm12_pulse_idx;

    // BOOST duty (0..100)
    int boost_duty_pct;

    // Akım hedef/ölçüm (UI: integer göster; içeride float)
    float current_set_ma;     // hedef
    float current_meas_ma;    // ölçülen (k*area_filt)
    float area;               // ham alan metriği
    float area_last;          // son ham alan metriği
    float area_filt;          // IIR filtreden sonra

    // Ölçek: I_meas = k_area_to_ma * area_filt
    // (CAL tuşuna basınca RAM’de güncellenir)
    float k_area_to_ma;

    // PI integrator
    float i_err;

    // OLED var mı?
    int oled_ok;

    portMUX_TYPE mux;
} app_state_t;

extern app_state_t g_app;
