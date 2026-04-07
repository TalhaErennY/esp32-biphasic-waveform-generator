#pragma once
#include <stdint.h>

void boost_pwm_init(int gpio);
void boost_pwm_set_duty_pct(int duty_pct);
int  boost_pwm_get_duty_pct(void);
