//
// Created by Zane Claes on 1/26/26.
//

#include "coord.h"

#include "esp_log.h"
#include "pumps.h"
#include "soil.h"
#include "esp_adc/adc_oneshot.h"

static const char *TAG = "coord";

static adc_oneshot_unit_handle_t adc_handle;

void shared_start() {
  adc_oneshot_unit_init_cfg_t init_cfg = {
    .unit_id = SHARED_ADC_UNIT,
    .ulp_mode = ADC_ULP_MODE_DISABLE,
  };
  ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_cfg, &adc_handle));
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
  pumps_start();
  soil_start();
}
