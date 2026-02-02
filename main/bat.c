#include "bat.h"
#include <stdint.h>

#include "coord.h"
#include "esp_log.h"
#include "esp_pm.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"

#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"

static const char *TAG = "bat";

static adc_cali_handle_t adc_cali_handle = NULL;
static adc_channel_t adc_channel = ADC_CHANNEL_0;

static struct BatteryLevel battery_level = {
  .voltage = VOLTAGE_MIN,
  .percent = 0,
};

void battery_start(void){
  ESP_LOGI(TAG, "Battery Starting...");
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
    .max_freq_mhz = CONFIG_ESP_DEFAULT_CPU_FREQ_MHZ, //80,
    .min_freq_mhz = CONFIG_ESP_DEFAULT_CPU_FREQ_MHZ, // 20,
// #if CONFIG_FREERTOS_USE_TICKLESS_IDLE
    .light_sleep_enable = true
// #endif
  };
  ESP_ERROR_CHECK(esp_pm_configure(&pm));
}

static float read_battery_voltage(void){
  int raw, mv;
  adc_read_avg(adc_channel, &raw);
  adc_cali_raw_to_voltage(adc_cali_handle, raw, &mv);
  return ((float)mv) * VBAT_DIVIDER_RATIO / 1000.0f;
}

static uint8_t battery_pct_from_voltage(float v) {
  if (v >= VOLTAGE_MAX) return 100;
  if (v <= VOLTAGE_MIN) return 0;
  return (uint8_t)((v - VOLTAGE_MIN) / (VOLTAGE_MAX - VOLTAGE_MIN) * 100.0f);
}

const struct BatteryLevel* update_battery(void) {
  battery_level.voltage = read_battery_voltage();
  battery_level.percent = battery_pct_from_voltage(battery_level.voltage);
  ESP_LOGI(TAG, "Bat %d%% (%fv)", battery_level.percent, battery_level.voltage);
  return &battery_level;
}

const struct BatteryLevel* get_battery(void) { return &battery_level; }