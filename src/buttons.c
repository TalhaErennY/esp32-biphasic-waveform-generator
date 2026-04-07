#include "buttons.h"
#include "driver/gpio.h"
#include "esp_err.h"

static buttons_pins_t s_pins;

void buttons_init(buttons_pins_t pins)
{
    s_pins = pins;
    gpio_config_t io = {0};
    io.mode = GPIO_MODE_INPUT;
    io.pull_up_en = GPIO_PULLUP_ENABLE;
    io.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io.intr_type = GPIO_INTR_DISABLE;

    io.pin_bit_mask =
        (1ULL << s_pins.freq) |
        (1ULL << s_pins.pulse) |
        (1ULL << s_pins.up) |
        (1ULL << s_pins.dn);

    ESP_ERROR_CHECK(gpio_config(&io));
}

int button_pressed(int gpio)
{
    return gpio_get_level((gpio_num_t)gpio) == 0; // active-low
}
