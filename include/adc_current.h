#pragma once
#include <stdint.h>

typedef struct {
    // “alan metriği” ham
    float area;
    // “alan metriği” IIR filtrelenmiş
    float area_filt;
} adc_area_result_t;

void adc_current_init(void);

// PWM12 callback'ten çağrılacak: “şu anda ölçüm penceresi aç”
void adc_current_request_capture(uint32_t window_us);

// ADC task handle içerde: sonucu app_state’e yazar
