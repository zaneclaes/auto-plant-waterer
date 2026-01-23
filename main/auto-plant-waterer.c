#include <esp_err.h>
#include <esp_log.h>
#include <esp_mac.h>
#include <nvs_flash.h>
#include <stdio.h>
#include <string.h>

#include "bat.h"
#include "cfg.h"
#include "wifi.h"
#include "zb.h"

static const char *TAG = "main";

void app_main(void) {
  cfg_start();
  // wifi_start();
  zb_start();
}
