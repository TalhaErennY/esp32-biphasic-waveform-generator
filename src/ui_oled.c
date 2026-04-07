#include "ui_oled.h"
#include <stdio.h>
#include <math.h>

#include "driver/i2c.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "ssd1306.h"

#include "app_state.h"
#include "app_config.h"

#define OLED_I2C_PORT     I2C_NUM_0
#define OLED_ADDR         0x3C
#define OLED_I2C_FREQ_HZ  400000

static const ssd1306_i2c_t g_oled = {
    .port = OLED_I2C_PORT,
    .addr = OLED_ADDR,
    .sda_gpio = OLED_SDA_GPIO,
    .scl_gpio = OLED_SCL_GPIO,
    .freq_hz = OLED_I2C_FREQ_HZ
};

static int i2c_device_present(i2c_port_t port, uint8_t addr)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_stop(cmd);
    esp_err_t err = i2c_master_cmd_begin(port, cmd, pdMS_TO_TICKS(50));
    i2c_cmd_link_delete(cmd);
    return (err == ESP_OK);
}

void ui_oled_init(void)
{
    ssd1306_i2c_init(&g_oled);

    int ok = i2c_device_present(g_oled.port, g_oled.addr);
    portENTER_CRITICAL(&g_app.mux);
    g_app.oled_ok = ok;
    portEXIT_CRITICAL(&g_app.mux);

    printf("OLED probe addr 0x%02X: %s\n", g_oled.addr, ok ? "OK" : "NOT FOUND");
    if (ok) ssd1306_init(&g_oled);
}

int ui_oled_is_ok(void)
{
    int ok;
    portENTER_CRITICAL(&g_app.mux);
    ok = g_app.oled_ok;
    portEXIT_CRITICAL(&g_app.mux);
    return ok;
}

void ui_oled_show_welcome(void)
{
    if (!ui_oled_is_ok()) return;
    ssd1306_clear(&g_oled);
    ssd1306_draw_text_8x8(&g_oled, 0, 1, "Baslatiliyor...");
    ssd1306_draw_text_8x8(&g_oled, 0, 3, "Tensprot v2");
    ssd1306_draw_text_8x8(&g_oled, 0, 5, "PI + ADC area");
}

static void render(int fi, int pi, int duty, int set_ma_i, int meas_ma_i, float k)
{
    char l0[24], l1[24], l2[24], l3[24], l4[24], l5[24];

    uint32_t f = g_pwm12_freq_hz_list[fi];
    uint32_t p = g_pwm12_pulse_us_list[pi];

    snprintf(l0, sizeof(l0), "PWM12 %luHz %luus", (unsigned long)f, (unsigned long)p);
    snprintf(l1, sizeof(l1), "BOOST duty:%d%%", duty);
    snprintf(l2, sizeof(l2), "Iset:%d mA", set_ma_i);
    snprintf(l3, sizeof(l3), "Imeas:%d mA", meas_ma_i);
    snprintf(l4, sizeof(l4), "k:%.4f", (double)k);
    snprintf(l5, sizeof(l5), "Rsense:%.0fohm", (double)RSENSE_OHMS);

    ssd1306_clear(&g_oled);
    ssd1306_draw_text_8x8(&g_oled, 0, 0, l0);
    ssd1306_draw_text_8x8(&g_oled, 0, 2, l1);
    ssd1306_draw_text_8x8(&g_oled, 0, 4, l2);
    ssd1306_draw_text_8x8(&g_oled, 0, 5, l3);
    ssd1306_draw_text_8x8(&g_oled, 0, 6, l4);
    ssd1306_draw_text_8x8(&g_oled, 0, 7, l5);
}

void ui_oled_task(void *arg)
{
    (void)arg;

    int last_hash = -1;

    while (1) {
        if (!ui_oled_is_ok()) {
            vTaskDelay(pdMS_TO_TICKS(500));
            continue;
        }

        int fi, pi, duty;
        float set_ma, meas_ma, k;

        portENTER_CRITICAL(&g_app.mux);
        fi   = g_app.pwm12_freq_idx;
        pi   = g_app.pwm12_pulse_idx;
        duty = g_app.boost_duty_pct;
        set_ma  = g_app.current_set_ma;
        meas_ma = g_app.current_meas_ma;
        k = g_app.k_area_to_ma;
        portEXIT_CRITICAL(&g_app.mux);

        int set_i  = (int)lroundf(set_ma);
        int meas_i = (int)lroundf(meas_ma);

        int h = fi*100000 + pi*10000 + duty*100 + set_i*10 + meas_i;
        if (h != last_hash) {
            last_hash = h;
            render(fi, pi, duty, set_i, meas_i, k);
        }

        vTaskDelay(pdMS_TO_TICKS(60));
    }
}
