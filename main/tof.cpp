#include "tof.h"

#include "esp_check.h"
#include "VL53l0X.h"
#include "driver/i2c.h"
#include "esp_log.h"

static const char *TAG = "tof";

static VL53L0X vl(TOF_I2C_PORT);

static struct WaterLevel water_level = {
  .mm = WATER_LEVEL_MAX_MM,
  .mm_clamped = WATER_LEVEL_MAX_MM,
  .percent = 0,
};

esp_err_t tof_start(void) {
  vl.i2cMasterInit(TOF_I2C_SDA_GPIO, TOF_I2C_SCL_GPIO);

  if (!vl.init()) {
    ESP_LOGE(TAG, "Failed to initialize VL53L0X :(");
    return ESP_FAIL;
  }

  // i2c_config_t conf = {
  //   .mode = I2C_MODE_MASTER,
  //   .sda_io_num = I2C_SDA_GPIO, // D4
  //   .scl_io_num = I2C_SCL_GPIO, // D5
  //   .sda_pullup_en = GPIO_PULLUP_ENABLE,
  //   .scl_pullup_en = GPIO_PULLUP_ENABLE,
  //   .master = {.clk_speed = I2C_FREQ_HZ},
  //   .clk_flags = 0,
  // };
  // ESP_ERROR_CHECK(i2c_param_config(I2C_NUM_0, &conf));
  // ESP_ERROR_CHECK(i2c_driver_install(I2C_NUM_0, conf.mode, 0, 0, 0));
  //
  // dev.port = I2C_NUM_0;
  // dev.address = TOF_I2C_ADDR;
  // dev.timeout_ms = 100;
  //
  // esp_err_t err = vl53l0x_simple_init(&dev);
  // if (err != ESP_OK) {
  //   ESP_LOGE(TAG, "VL53L0X init failed: %s", esp_err_to_name(err));
  //   return err;
  // }
  //
  // ESP_LOGI(TAG, "VL53L0X started");
  return ESP_OK;
}

const struct WaterLevel* tof_update(void) {
  uint16_t mm = 0;
  bool res = vl.read(&mm);
  if (res) {
    water_level.mm = mm;
  } else {
    ESP_LOGW(TAG, "VL53L0X read failed");
    return &water_level;
  }

  // Clamp + map distance->percent (smaller distance = more full)
  water_level.mm_clamped = water_level.mm;
  if (water_level.mm_clamped < WATER_LEVEL_MIN_MM) water_level.mm_clamped = WATER_LEVEL_MIN_MM;
  if (water_level.mm_clamped > WATER_LEVEL_MAX_MM) water_level.mm_clamped = WATER_LEVEL_MAX_MM;

  uint32_t range = (uint32_t)(WATER_LEVEL_MAX_MM - WATER_LEVEL_MIN_MM);
  uint32_t from_min = (uint32_t)(water_level.mm_clamped - WATER_LEVEL_MIN_MM);
  uint32_t pct = (100U * (range - from_min) + (range / 2)) / range; // rounded
  if (pct > 100U) pct = 100U;

  water_level.percent = (uint8_t)pct;

  ESP_LOGI(TAG, "Water level: %u mm -> %u%%", (unsigned)water_level.mm, (unsigned)water_level.percent);
  return &water_level;
}
