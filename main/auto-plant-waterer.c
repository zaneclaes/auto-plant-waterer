#include <esp_log.h>
#include <string.h>
#include <sys/unistd.h>

#include "bat.h"
#include "cfg.h"
#include "coord.h"
#include "portmacro.h"
#include "pumps.h"
#include "soil.h"
#include "tof.h"
#include "zb.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "main";

void app_main(void) {
  ESP_LOGI(TAG, "Starting...");
  vTaskDelay(pdMS_TO_TICKS(500));
  ESP_LOGI(TAG, "Starting2...");
  // enable_power_management();
  shared_start();
  cfg_start();
  battery_start();
  set_cfg_flag(CFG_FLAG_TOF, tof_start() == ESP_OK);
  pumps_start();
  soil_start();
  // wifi_start();

  // gpio_config_t cfg = {
  //   .mode = GPIO_MODE_OUTPUT,
  //   .pin_bit_mask =
  //     (1ULL << PIN_PUMP1) | (1ULL << PIN_PUMP2) | (1ULL << PIN_PUMP3) |
  //     (1ULL << PIN_SOIL1) | (1ULL << PIN_SOIL2) | (1ULL << PIN_SOIL3) | (1ULL << PIN_SOIL_ON),
  //   .pull_down_en = GPIO_PULLDOWN_DISABLE,
  //   .pull_up_en = GPIO_PULLUP_DISABLE,
  //   .intr_type = GPIO_INTR_DISABLE,
  // };
  // ESP_ERROR_CHECK(gpio_config(&cfg));

  zb_start();
  cfg_save();
  ESP_LOGI(TAG, "Started %d zones.", get_num_zones());
}
