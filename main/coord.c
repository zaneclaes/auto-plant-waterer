//
// Created by Zane Claes on 1/26/26.
//

#include "coord.h"

#include "esp_log.h"
#include "pumps.h"
#include "soil.h"
#include "driver/gpio.h"
#include "esp_adc/adc_oneshot.h"
#include "soc/gpio_num.h"

static const char *TAG = "coord";

#define PIN_LED     GPIO_NUM_15

static adc_oneshot_unit_handle_t adc_handle;

void shared_start() {
  ESP_LOGI(TAG, "Shared Start...");
  gpio_config_t cfg = {
    .pin_bit_mask = 1ULL << PIN_LED,
    .mode = GPIO_MODE_OUTPUT,
    .pull_up_en = GPIO_PULLUP_DISABLE,
    .pull_down_en = GPIO_PULLDOWN_DISABLE,
    .intr_type = GPIO_INTR_DISABLE,
  };
  ESP_ERROR_CHECK(gpio_config(&cfg));
  set_led(true);

  adc_oneshot_unit_init_cfg_t init_cfg = {
    .unit_id = SHARED_ADC_UNIT,
    .ulp_mode = ADC_ULP_MODE_DISABLE,
  };
  ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_cfg, &adc_handle));
}

void set_led(bool on) {
  gpio_set_level(PIN_LED, on ? 0 : 1);
}

adc_channel_t adc_start(int pin) {
  adc_unit_t unit;
  adc_channel_t ch;
  ESP_ERROR_CHECK(adc_oneshot_io_to_channel(pin, &unit, &ch));
  if (unit != SHARED_ADC_UNIT) ESP_LOGW(TAG, "GPIO%d mapped to unexpected ADC unit %d", pin, (int)unit);

  adc_oneshot_chan_cfg_t cfg = {
    .atten = ADC_ATTEN_DB_12, // widest range; good for 0-3.3V-ish sensors
    .bitwidth = ADC_BITWIDTH_DEFAULT,
  };
  ESP_ERROR_CHECK(adc_oneshot_config_channel(adc_handle, ch, &cfg));
  return ch;
}

esp_err_t adc_shared_read(const adc_channel_t ch, int* raw) {
  return adc_oneshot_read(adc_handle, ch, raw);
}

void on_joined() {
  set_led(false);
}
