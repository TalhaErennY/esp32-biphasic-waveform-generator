#include "app_state.h"
#include "app_config.h"

app_state_t g_app = {
    .pwm12_freq_idx   = 6,
    .pwm12_pulse_idx  = 0,

    .boost_duty_pct   = 0,

    .current_set_ma   = 0.0f,
    .current_meas_ma  = 0.0f,

    .area             = 0.0f,
    .area_last        = 0.0f,
    .area_filt        = 0.0f,

    // İlk başta kaba bir ölçek. CAL tuşuyla düzeltilecek.
    .k_area_to_ma     = 0.01f,

    .i_err            = 0.0f,

    .oled_ok          = 0,
    .mux              = portMUX_INITIALIZER_UNLOCKED
};
