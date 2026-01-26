#include "bat.h"

#include <stddef.h>
#include <stdint.h>

#include "coord.h"
#include "esp_pm.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"

#include "driver/gpio.h"

static adc_cali_handle_t adc_cali_handle = NULL;
static adc_channel_t adc_channel = ADC_CHANNEL_0;

static struct BatteryLevel battery_level = {
  .voltage = VOLTAGE_MIN,
  .percent = 0,
};

void battery_start(void)
{
  adc_channel = adc_start(PIN_BAT);

  adc_cali_curve_fitting_config_t cali_cfg = {
    .unit_id = SHARED_ADC_UNIT,
    .chan = adc_channel,
    .atten = ADC_ATTEN_DB_12,
    .bitwidth = ADC_BITWIDTH_DEFAULT,
  };

  esp_err_t err = adc_cali_create_scheme_curve_fitting(&cali_cfg, &adc_cali_handle);
  if (err != ESP_OK) {
    adc_cali_handle = NULL;
  }
}

void enable_power_management(void) {
  esp_pm_config_t pm = {
    .max_freq_mhz = 80,
    .min_freq_mhz = 10,
    .light_sleep_enable = true,
  };
  ESP_ERROR_CHECK(esp_pm_configure(&pm));
}

static float read_battery_voltage(void){
  int mv_sum = 0;

  for (int i = 0; i < 8; i++) {
    int raw, mv; // average 8 samples for noise reduction (instead of using capacitor)
    adc_shared_read(adc_channel, &raw);
    adc_cali_raw_to_voltage(adc_cali_handle, raw, &mv);
    mv_sum += mv;
  }

  float v_adc = (mv_sum / 8) / 1000.0f;
  return v_adc * VBAT_DIVIDER_RATIO;
}

static uint8_t battery_pct_from_voltage(float v) {
  if (v >= VOLTAGE_MAX) return 100;
  if (v <= VOLTAGE_MIN) return 0;
  return (uint8_t)((v - VOLTAGE_MIN) / (VOLTAGE_MAX - VOLTAGE_MIN) * 100.0f);
}

const struct BatteryLevel* battery_update(void) {
  battery_level.voltage = read_battery_voltage();
  battery_level.percent = battery_pct_from_voltage(battery_level.voltage);
  return &battery_level;
}