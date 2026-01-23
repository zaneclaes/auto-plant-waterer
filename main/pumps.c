//
// Created by Zane Claes on 1/21/26.
//

#include "pumps.h"

#include "esp_log.h"
#include "driver/gpio.h"

static const char *TAG = "pump";

static const gpio_num_t s_pump_gpios[3] = { PUMP1_GPIO, PUMP2_GPIO, PUMP3_GPIO };

void pumps_start(void) {
  gpio_config_t cfg = {
    .mode = GPIO_MODE_OUTPUT,
    .pin_bit_mask =
        (1ULL << PUMP1_GPIO) |
        (1ULL << PUMP2_GPIO) |
        (1ULL << PUMP3_GPIO),
    .pull_down_en = GPIO_PULLDOWN_DISABLE,
    .pull_up_en = GPIO_PULLUP_DISABLE,
    .intr_type = GPIO_INTR_DISABLE,
};
  ESP_ERROR_CHECK(gpio_config(&cfg));

  /* Default OFF */
  gpio_set_level(PUMP1_GPIO, 0);
  gpio_set_level(PUMP2_GPIO, 0);
  gpio_set_level(PUMP3_GPIO, 0);
}

void pump_set(const uint8_t idx, const bool on) {
  gpio_set_level(s_pump_gpios[idx], on ? 1 : 0);
  ESP_LOGI(TAG, "Pump endpoint %u -> %s (GPIO %d)", idx, on ? "ON" : "OFF", (int)s_pump_gpios[idx]);
}