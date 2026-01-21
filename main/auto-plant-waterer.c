#include <esp_err.h>
#include <esp_log.h>
#include <esp_mac.h>
#include <nvs_flash.h>
#include <stdio.h>
#include <string.h>

#include "cfg.h"
#include "wifi.h"

static const char *TAG = "main";

void app_main(void) {
  esp_err_t ret = nvs_flash_init();
  if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    ESP_ERROR_CHECK(nvs_flash_erase());
    ESP_ERROR_CHECK(nvs_flash_init());
  }

  wifi_start();
}
