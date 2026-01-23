#include "tof.h"

#include "esp_check.h"
#include "driver/i2c.h"
#include <cmath>

static const char *TAG = "tof";

static struct WaterLevel water_level = {
  .mm = WATER_LEVEL_MAX_MM,
  .mm_clamped = WATER_LEVEL_MAX_MM,
  .percent = 0,
};


void tof_start(void) {
  // // ---- I2C init ----
  // i2c_config_t conf = {};
  // conf.mode = I2C_MODE_MASTER;
  // conf.sda_io_num = I2C_SDA_GPIO;
  // conf.scl_io_num = I2C_SCL_GPIO;
  // conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
  // conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
  // conf.master.clk_speed = I2C_FREQ_HZ;
  //
  // ESP_ERROR_CHECK(i2c_param_config(I2C_PORT, &conf));
  // ESP_ERROR_CHECK(i2c_driver_install(I2C_PORT, conf.mode, 0, 0, 0));
  //
  // // ---- VL53L0X init ----
  // if (s_vl53l) {
  //   return;
  // }
  //
  // // espp::Vl53l expects callbacks for raw I2C read/write
  // s_vl53l = new espp::Vl53l(espp::Vl53l::Config{
  //     .device_address = espp::Vl53l::DEFAULT_ADDRESS, // 0x29 for VL53L0X :contentReference[oaicite:2]{index=2}
  //     .write = vl53l_i2c_write,
  //     .read  = vl53l_i2c_read,
  //     .log_level = espp::Logger::Verbosity::WARN,
  // });
  //
  // std::error_code ec;
  //
  // // Timing budget must be <= inter-measurement period. The docs example shows this pattern. :contentReference[oaicite:3]{index=3}
  // if (!s_vl53l->set_timing_budget_ms(20, ec)) {
  //   ESP_LOGE(TAG, "VL53L0X: set_timing_budget_ms failed: %s", ec.message().c_str());
  //   return;
  // }
  // ec.clear();
  // if (!s_vl53l->set_inter_measurement_period_ms(25, ec)) {
  //   ESP_LOGE(TAG, "VL53L0X: set_inter_measurement_period_ms failed: %s", ec.message().c_str());
  //   return;
  // }
  // ec.clear();
  // if (!s_vl53l->start_ranging(ec)) {
  //   ESP_LOGE(TAG, "VL53L0X: start_ranging failed: %s", ec.message().c_str());
  //   return;
  // }

  ESP_LOGI(TAG, "VL53L0X started.");
}

const struct WaterLevel* tof_update(){
  // Default: if sensor isn't ready, keep last value
  // if (!s_vl53l) {
  //   ESP_LOGW(TAG, "VL53L0X not started yet.");
  //   return &water_level;
  // }
  //
  // std::error_code ec;
  //
  // // Wait briefly for fresh data (don’t block forever)
  // // The docs example polls is_data_ready then clears interrupt. :contentReference[oaicite:4]{index=4}
  // constexpr int max_wait_ms = 30;
  // int waited = 0;
  // while (!s_vl53l->is_data_ready(ec) && waited < max_wait_ms) {
  //   vTaskDelay(pdMS_TO_TICKS(1));
  //   waited += 1;
  // }
  // ec.clear();
  //
  // // Clear interrupt so next measurement can arrive
  // if (!s_vl53l->clear_interrupt(ec)) {
  //   ESP_LOGW(TAG, "VL53L0X: clear_interrupt failed: %s", ec.message().c_str());
  //   ec.clear();
  // }
  //
  // // Read distance
  // float meters = s_vl53l->get_distance_meters(ec);
  // if (ec) {
  //   ESP_LOGW(TAG, "VL53L0X: get_distance_meters failed: %s", ec.message().c_str());
  //   ec.clear();
  //   return &water_level; // keep last
  // }
  //
  // // Convert to mm (clamp to sane range)
  // int mm = std::lround(meters * 1000.0f + 0.5f);
  // if (mm < 0) mm = 0;
  // if (mm > 4000) mm = 4000; // VL53L0X is ~2m typical; clamp extra just in case
  int mm = 0;

  water_level.mm = (uint16_t)mm;

  if (water_level.mm < WATER_LEVEL_MIN_MM) water_level.mm = WATER_LEVEL_MAX_MM;

  water_level.mm_clamped = water_level.mm;
  if (water_level.mm_clamped < WATER_LEVEL_MIN_MM) water_level.mm_clamped = WATER_LEVEL_MIN_MM;
  if (water_level.mm_clamped > WATER_LEVEL_MAX_MM) water_level.mm_clamped = WATER_LEVEL_MAX_MM;

  /* Map distance->percent (smaller distance = more full) */
  uint32_t range = (uint32_t)(WATER_LEVEL_MAX_MM - WATER_LEVEL_MIN_MM);
  uint32_t from_min = (uint32_t)(water_level.mm_clamped - WATER_LEVEL_MIN_MM);

  uint32_t pct = (100U * (range - from_min) + (range/2)) / range; // rounded
  if (pct > 100U) pct = 100U;
  water_level.percent = pct;

  ESP_LOGI(TAG, "Water level: %u mm -> %d%%", (unsigned)water_level.mm, (int)water_level.percent);
  return &water_level;
}