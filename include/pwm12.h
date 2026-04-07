#pragma once
#include <stdint.h>

// PWM12 biphasic pulse generator (software timing, low freq ok)

// Başlat
void pwm12_init(int gpio1, int gpio2, uint32_t gap_us);

// Ayarları uygula
void pwm12_set(uint32_t freq_hz, uint32_t pulse_us);

// ADC “alan” ölçümü için senkron bildirim:
// Impuls başladığında bir callback çağırır (pencere_us param ile).
typedef void (*pwm12_pulse_cb_t)(uint32_t window_us, void *user);
void pwm12_set_pulse_callback(pwm12_pulse_cb_t cb, void *user);
