#include "boost_pwm.h"
#include <stdio.h>
#include "driver/ledc.h"
#include "esp_err.h"

#include "app_config.h"

static int s_gpio = -1;
static int s_duty_pct = 0;
static uint32_t s_max_duty = 0;

void boost_pwm_init(int gpio)
{
    s_gpio = gpio;

    ESP_ERROR_CHECK(ledc_timer_config(&(ledc_timer_config_t){
        .speed_mode       = BOOST_LEDC_MODE,
        .timer_num        = BOOST_LEDC_TIMER,
        .duty_resolution  = BOOST_LEDC_RES,
        .freq_hz          = BOOST_FREQ_HZ,
        .clk_cfg          = LEDC_AUTO_CLK
    }));

    ESP_ERROR_CHECK(ledc_channel_config(&(ledc_channel_config_t){
        .gpio_num   = s_gpio,
        .speed_mode = BOOST_LEDC_MODE,
        .channel    = BOOST_LEDC_CHANNEL,
        .timer_sel  = BOOST_LEDC_TIMER,
        .duty       = 0,
        .hpoint     = 0
    }));

    s_max_duty = (1U << BOOST_LEDC_RES) - 1U;

    printf("[BOOST] LEDC init %d Hz, gpio=%d, timer=%d, ch=%d\n",
           BOOST_FREQ_HZ, s_gpio, BOOST_LEDC_TIMER, BOOST_LEDC_CHANNEL);
}

static uint32_t pct_to_ticks(int pct)
{
    if (pct < 0) pct = 0;
    if (pct > 100) pct = 100;
    return (uint32_t)((uint64_t)pct * s_max_duty / 100ULL);
}

void boost_pwm_set_duty_pct(int duty_pct)
{
    if (duty_pct < BOOST_DUTY_MIN_PCT) duty_pct = BOOST_DUTY_MIN_PCT;
    if (duty_pct > BOOST_DUTY_MAX_PCT) duty_pct = BOOST_DUTY_MAX_PCT;

    s_duty_pct = duty_pct;

    uint32_t ticks = pct_to_ticks(duty_pct);
    ESP_ERROR_CHECK(ledc_set_duty(BOOST_LEDC_MODE, BOOST_LEDC_CHANNEL, ticks));
    ESP_ERROR_CHECK(ledc_update_duty(BOOST_LEDC_MODE, BOOST_LEDC_CHANNEL));
}

int boost_pwm_get_duty_pct(void)
{
    return s_duty_pct;
}
