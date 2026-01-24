#include <esp_log.h>
#include <string.h>
#include <sys/unistd.h>

#include "bat.h"
#include "cfg.h"
#include "tof.h"
#include "zb.h"

static const char *TAG = "main";

void app_main(void) {
  // enable_power_management();
  cfg_start();
  tof_start();
  // wifi_start();
  zb_start();
}
