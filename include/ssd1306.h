#pragma once

#include <stdint.h>
#include "driver/i2c.h"

typedef struct {
    i2c_port_t port;
    uint8_t addr;         // genelde 0x3C
    int sda_gpio;
    int scl_gpio;
    uint32_t freq_hz;     // 100k/400k
} ssd1306_i2c_t;

void ssd1306_i2c_init(const ssd1306_i2c_t *cfg);
void ssd1306_init(const ssd1306_i2c_t *cfg);

void ssd1306_clear(const ssd1306_i2c_t *cfg);

// page: 0..7 (her page 8 piksel yüksek)
// col: 0..127 piksel sütunu (8x8 karakter basarken col 8 artar)
void ssd1306_draw_text_8x8(const ssd1306_i2c_t *cfg, int col, int page, const char *text);
