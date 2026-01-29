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

static void warning_task(void* pv) {
  vTaskDelay(pdMS_TO_TICKS(60000));
  while (true) {
    const struct WaterLevel* wl = get_water_level();
    const struct BatteryLevel* bl = get_battery();

    bool low_bat = bl->percent < 10;
    bool low_wtr = get_cfg_flag(CFG_FLAG_TOF) && wl->percent < 10;

    if (low_bat || low_wtr) {
      if (low_bat) {
        set_led(true);
        vTaskDelay(pdMS_TO_TICKS(500));
        set_led(false);
        vTaskDelay(pdMS_TO_TICKS(500));
      } else {
        vTaskDelay(pdMS_TO_TICKS(1000));
      }
      if (low_wtr) {
        set_led(true);
        vTaskDelay(pdMS_TO_TICKS(1500));
        set_led(false);
        vTaskDelay(pdMS_TO_TICKS(500));
      } else {
        vTaskDelay(pdMS_TO_TICKS(2000));
      }
      vTaskDelay(pdMS_TO_TICKS(12 * 1000));
    } else {
      vTaskDelay(pdMS_TO_TICKS(60 * 60 * 1000));
    }
  }
}

void app_main(void) {
  ESP_LOGI(TAG, "Starting...");
  vTaskDelay(pdMS_TO_TICKS(500));
  ESP_LOGI(TAG, "Starting2...");
  shared_start();
  cfg_start();
  battery_start();
  set_cfg_flag(CFG_FLAG_TOF, tof_start() == ESP_OK);
  pumps_start();
  soil_start();
  // wifi_start();
  cfg_save();

  enable_power_management();
  pm_lock();
  zb_start();
  xTaskCreate(warning_task, "warning_task", 0x1000, NULL, 2, NULL);

  ESP_LOGI(TAG, "Started %d zones.", get_num_zones());
}
