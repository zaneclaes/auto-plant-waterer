//
// Created by Zane Claes on 1/21/26.
//

#ifndef AUTO_PLANT_WATERER_NVS_H
#define AUTO_PLANT_WATERER_NVS_H

#include <esp_err.h>

#define NVS_NAMESPACE      "wifi_cfg"
#define KEY_STA_SSID       "sta_ssid"
#define KEY_STA_PASS       "sta_pass"
#define KEY_AP_SSID        "ap_ssid"
#define KEY_AP_PASS        "ap_pass"

// AP defaults (if nothing saved)
#define DEFAULT_AP_SSID    "planter"
#define DEFAULT_AP_PASS    "plant123"     // >= 8 chars for WPA2
#define DEFAULT_AP_CHANNEL 6
#define DEFAULT_AP_MAX_CONN 4

#define MAX_SSID_LEN       32
#define MAX_PASS_LEN       64

typedef struct {
  char sta_ssid[MAX_SSID_LEN + 1];
  char sta_pass[MAX_PASS_LEN + 1];
  char ap_ssid[MAX_SSID_LEN + 1];
  char ap_pass[MAX_PASS_LEN + 1];
} wifi_cfg_t;

esp_err_t load_wifi_cfg(wifi_cfg_t *cfg);
esp_err_t save_wifi_cfg(const wifi_cfg_t *cfg);

#endif //AUTO_PLANT_WATERER_NVS_H