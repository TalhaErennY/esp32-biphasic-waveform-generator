#include "ssd1306.h"
#include "font8x8_basic.h"

#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_err.h"

// -------- low-level helpers --------
static void oled_cmd(const ssd1306_i2c_t *cfg, uint8_t cmd)
{
    i2c_cmd_handle_t h = i2c_cmd_link_create();
    i2c_master_start(h);
    i2c_master_write_byte(h, (cfg->addr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(h, 0x00, true); // command
    i2c_master_write_byte(h, cmd, true);
    i2c_master_stop(h);
    ESP_ERROR_CHECK(i2c_master_cmd_begin(cfg->port, h, pdMS_TO_TICKS(100)));
    i2c_cmd_link_delete(h);
}

static void oled_data(const ssd1306_i2c_t *cfg, const uint8_t *data, size_t len)
{
    i2c_cmd_handle_t h = i2c_cmd_link_create();
    i2c_master_start(h);
    i2c_master_write_byte(h, (cfg->addr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(h, 0x40, true); // data
    i2c_master_write(h, (uint8_t*)data, len, true);
    i2c_master_stop(h);
    ESP_ERROR_CHECK(i2c_master_cmd_begin(cfg->port, h, pdMS_TO_TICKS(100)));
    i2c_cmd_link_delete(h);
}

void ssd1306_i2c_init(const ssd1306_i2c_t *cfg)
{
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = cfg->sda_gpio,
        .scl_io_num = cfg->scl_gpio,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = cfg->freq_hz
    };

    ESP_ERROR_CHECK(i2c_param_config(cfg->port, &conf));
    ESP_ERROR_CHECK(i2c_driver_install(cfg->port, conf.mode, 0, 0, 0));
}

void ssd1306_init(const ssd1306_i2c_t *cfg)
{
    // küçük bekleme iyi gelir
    vTaskDelay(pdMS_TO_TICKS(50));

    oled_cmd(cfg, 0xAE); // display off
    oled_cmd(cfg, 0x20); oled_cmd(cfg, 0x00); // horizontal addressing mode
    oled_cmd(cfg, 0xB0); // page 0
    oled_cmd(cfg, 0xC8); // COM scan direction
    oled_cmd(cfg, 0x00); // low col
    oled_cmd(cfg, 0x10); // high col
    oled_cmd(cfg, 0x40); // start line
    oled_cmd(cfg, 0x81); oled_cmd(cfg, 0x7F); // contrast
    oled_cmd(cfg, 0xA1); // segment remap
    oled_cmd(cfg, 0xA6); // normal display
    oled_cmd(cfg, 0xA8); oled_cmd(cfg, 0x3F); // multiplex 1/64
    oled_cmd(cfg, 0xA4); // display follows RAM
    oled_cmd(cfg, 0xD3); oled_cmd(cfg, 0x00); // display offset
    oled_cmd(cfg, 0xD5); oled_cmd(cfg, 0x80); // clock div
    oled_cmd(cfg, 0xD9); oled_cmd(cfg, 0xF1); // pre-charge
    oled_cmd(cfg, 0xDA); oled_cmd(cfg, 0x12); // com pins
    oled_cmd(cfg, 0xDB); oled_cmd(cfg, 0x40); // vcomh
    oled_cmd(cfg, 0x8D); oled_cmd(cfg, 0x14); // charge pump
    oled_cmd(cfg, 0xAF); // display on
}

void ssd1306_clear(const ssd1306_i2c_t *cfg)
{
    uint8_t zeros[128];
    memset(zeros, 0, sizeof(zeros));

    for (int page = 0; page < 8; page++) {
        oled_cmd(cfg, 0xB0 + page);
        oled_cmd(cfg, 0x00);
        oled_cmd(cfg, 0x10);
        oled_data(cfg, zeros, sizeof(zeros));
    }
}

// Font tablosu satır bazlı (row bitmap). SSD1306 page mode sütun ister.
// Bu fonksiyon 8x8'de row->column transpoze yapıp gönderir.
void ssd1306_draw_text_8x8(const ssd1306_i2c_t *cfg, int col, int page, const char *text)
{
    if (!text) return;
    if (page < 0 || page > 7) return;
    if (col < 0) col = 0;
    if (col > 127) return;

    oled_cmd(cfg, 0xB0 + page);
    oled_cmd(cfg, 0x00 + (col & 0x0F));
    oled_cmd(cfg, 0x10 + ((col >> 4) & 0x0F));

    while (*text && col <= 120) {
        unsigned char ch = (unsigned char)*text++;
        if (ch < 0x20 || ch > 0x7F) ch = '?';

        const uint8_t *rows = font8x8_basic[ch - 0x20];
        uint8_t cols[8] = {0};

        // transpose: rows[r] bit(7-c) => cols[c] bit(r)
        for (int r = 0; r < 8; r++) {
            for (int c = 0; c < 8; c++) {
                if (rows[r] & (1 << (7 - c))) {
                    cols[c] |= (1 << r);
                }
            }
        }

        oled_data(cfg, cols, 8);
        col += 8;
    }
}
