#pragma once
#include <stdint.h>

typedef struct {
    int freq;
    int pulse;
    int up;
    int dn;
} buttons_pins_t;

void buttons_init(buttons_pins_t pins);
int  button_pressed(int gpio);  // active-low
