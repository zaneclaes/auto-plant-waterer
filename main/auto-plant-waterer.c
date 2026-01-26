#include <esp_log.h>
#include <string.h>
#include <sys/unistd.h>

#include "bat.h"
#include "cfg.h"
#include "pumps.h"
#include "soil.h"
#include "tof.h"
#include "zb.h"

static const char *TAG = "main";

void app_main(void) {
  ESP_LOGI(TAG, "Startup...");
  // enable_power_management();
  cfg_start();
  tof_start();
  pumps_start();
  soil_start();
  while (true) {
    soil_get_level(0);
    sleep(1);
  }
  // wifi_start();
  zb_start();
}
