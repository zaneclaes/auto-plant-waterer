#include "bat.h"

#include <stddef.h>
#include <stdint.h>

#include "esp_pm.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"

static adc_oneshot_unit_handle_t adc_handle;
static adc_cali_handle_t adc_cali_handle = NULL;

static struct BatteryLevel battery_level = {
  .voltage = VOLTAGE_MIN,
  .percent = 0,
};

void battery_start(void)
{
  adc_oneshot_unit_init_cfg_t init_cfg = {
    .unit_id = ADC_UNIT_1,
};
  ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_cfg, &adc_handle));

  adc_oneshot_chan_cfg_t chan_cfg = {
    .atten = ADC_ATTEN_DB_12,      // Required for >3V range
    .bitwidth = ADC_BITWIDTH_DEFAULT,
};
  ESP_ERROR_CHECK(adc_oneshot_config_channel(adc_handle, ADC_CHANNEL_0, &chan_cfg));

  /* ---- ADC calibration ---- */
  adc_cali_curve_fitting_config_t cali_cfg = {
    .unit_id = ADC_UNIT_1,
    .chan = ADC_CHANNEL_0,
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
    adc_oneshot_read(adc_handle, ADC_CHANNEL_0, &raw);
    adc_cali_raw_to_voltage(adc_cali_handle, raw, &mv);
    mv_sum += mv;
  }

  float v_adc = (mv_sum / 8) / 1000.0f;
  return v_adc * VBAT_DIVIDER_RATIO;
}

static uint8_t battery_pct_from_voltage(float v) {
  if (v >= VOLTAGE_MAX) return 100;
  if (v <= VOLTAGE_MIN) return 0;

  // Simple Li-ion curve approximation
  return (uint8_t)((v - VOLTAGE_MIN) / (VOLTAGE_MAX - VOLTAGE_MIN) * 100.0f);
}

const struct BatteryLevel* battery_update(void) {
  battery_level.voltage = read_battery_voltage();
  battery_level.percent = battery_pct_from_voltage(battery_level.voltage);
  return &battery_level;
}
