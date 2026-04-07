#pragma once
#include <stdint.h>

void current_ctrl_init(void);
void current_ctrl_notify_pulse_window(uint32_t win_us, void *user); // pwm12 callback
