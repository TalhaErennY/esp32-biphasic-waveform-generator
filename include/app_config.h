#pragma once

#include "esp_adc/adc_oneshot.h"

// ----------------- PWM12 config -----------------
#define PWM12_GPIO           18
#define PWM12_DEFAULT_FREQ   100
#define PWM12_DEFAULT_PULSE_US  200
#define PWM12_FREQ_COUNT     5
#define PWM12_PULSE_COUNT    8

static const int PWM12_FREQ_LIST[PWM12_FREQ_COUNT] = { 50, 100, 200, 500, 1000 };
static const int PWM12_PULSE_LIST[PWM12_PULSE_COUNT] = { 50, 100, 200, 300, 500, 800, 1000, 1500 };

// ----------------- BOOST PWM config -----------------
#define BOOST_PWM_GPIO       17
#define BOOST_PWM_FREQ_HZ    20000

#define BOOST_DUTY_MIN_PCT   0
#define BOOST_DUTY_MAX_PCT   95

// ----------------- Buttons -----------------
#define BTN_UP_GPIO          12
#define BTN_DN_GPIO          13
#define BTN_MODE_GPIO        14
#define BTN_CAL_GPIO         15

#define BTN_DEBOUNCE_MS      30
#define BTN_LONG_MS          800

// ----------------- Current control -----------------
#define CURRENT_STEP_MA      0.5f     // butonla 0.5mA adım
#define CURRENT_MIN_MA       0.0f
#define CURRENT_MAX_MA       30.0f

// PI kontrol (STABİL başlangıç değerleri)
// Not: err mA cinsinden olduğu için Kp büyük olursa duty zıplar.
#define PI_KP                0.002f
#define PI_KI                0.2f
#define PI_I_CLAMP           5000.0f   // integrator clamp (mA*s)

// Alan ölçümü / filtre
#define AREA_IIR_ALPHA       0.05f     // daha stabil (0..1)

// ----------------- ADC config -----------------
// !!! BURAYI KENDİ PINİNE GÖRE AYARLA !!!
// ESP32-S3 ADC1 önerilir.
// Örn: QT Py ESP32-S3'te GPIO1 ADC1_CH0 olabilir (board'a göre değişir).
#define ADC_UNIT_USED        ADC_UNIT_1
#define ADC_CHANNEL_USED     ADC_CHANNEL_0

// Attenuation / extra window
#define ADC_ATTEN_USED       ADC_ATTEN_DB_11
#define ADC_WIN_EXTRA_US     0
