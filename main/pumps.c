#include "pumps.h"
#include "cfg.h"

#include "esp_check.h"
#include "esp_log.h"
#include "driver/gpio.h"

static const char *TAG = "pump";

static const gpio_num_t s_pump_gpios[3] = { PIN_PUMP1, PIN_PUMP2, PIN_PUMP3 };

void pumps_start(void) {
  gpio_config_t cfg = {
    .mode = GPIO_MODE_OUTPUT,
    .pin_bit_mask = 0,
    .pull_down_en = GPIO_PULLDOWN_DISABLE,
    .pull_up_en = GPIO_PULLUP_DISABLE,
    .intr_type = GPIO_INTR_DISABLE,
  };
  if (NUM_ZONES >= 1) cfg.pin_bit_mask |= (1ULL << PIN_PUMP1);
  if (NUM_ZONES >= 2) cfg.pin_bit_mask |= (1ULL << PIN_PUMP2);
  if (NUM_ZONES >= 3) cfg.pin_bit_mask |= (1ULL << PIN_PUMP3);
  ESP_ERROR_CHECK(gpio_config(&cfg));

  if (NUM_ZONES >= 1) gpio_set_level(PIN_PUMP1, 0);
  if (NUM_ZONES >= 2) gpio_set_level(PIN_PUMP2, 0);
  if (NUM_ZONES >= 3) gpio_set_level(PIN_PUMP3, 0);
}

void pump_set(const uint8_t idx, const bool on) {
  if (idx >= NUM_ZONES) {
    ESP_LOGW(TAG, "Pump #%u is not enabled (%d zones)", idx, NUM_ZONES);
    return;
  }
  ESP_LOGI(TAG, "Pump #%u -> %s (GPIO %d)", idx, on ? "ON" : "OFF", (int)s_pump_gpios[idx]);
  gpio_set_level(s_pump_gpios[idx], on ? 1 : 0);
}