#include <esp_err.h>
#include <esp_log.h>
#include "esp_pm.h"
#include <string.h>

#include "cfg.h"
#include "bat.h"
#include "pumps.h"
#include "tof.h"
#include "wifi.h"
#include "zb.h"

static const char *TAG = "main";

void app_main(void) {
  ESP_LOGI(TAG, "Entrypoint...");
  // enable_power_management();
  // cfg_start();
  // battery_start();
  // pumps_start();
  // tof_start();
  // wifi_start();
  zb_start();
  ESP_LOGI(TAG, "Startup complete.");
}
