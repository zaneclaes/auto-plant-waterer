//
// Created by Zane Claes on 1/21/26.
//

#ifndef AUTO_PLANT_WATERER_NVS_H
#define AUTO_PLANT_WATERER_NVS_H

#ifdef __cplusplus
extern "C" {
#endif

#include <esp_err.h>
#include "rom/secure_boot.h"

#define DEF_NUM_ZONES         3 // How many pumps/soil sensors?

#define ZB_MANUFACTURER_NAME "\x0b" "inZania LLC"
#define ZB_MODEL_IDENTIFIER  "\x13" "Auto Plant Waterer"

#define NVS_NAMESPACE      "wifi_cfg"
#define KEY_STA_SSID       "sta_ssid"
#define KEY_STA_PASS       "sta_pass"
#define KEY_AP_SSID        "ap_ssid"
#define KEY_AP_PASS        "ap_pass"
#define KEY_FLAGS          "flags"
#define KEY_NUM_ZONES       "num_zones"

typedef enum {
  CFG_FLAG_TOF   = (1U << 0),
} cfg_flag_t;

// AP defaults (if nothing saved)
#define DEFAULT_AP_SSID     "planter"
#define DEFAULT_AP_PASS     "plant123"     // >= 8 chars for WPA2
#define DEFAULT_AP_CHANNEL  6
#define DEFAULT_AP_MAX_CONN 4

#define MAX_SSID_LEN       32
#define MAX_PASS_LEN       64

typedef struct {
  char sta_ssid[MAX_SSID_LEN + 1];
  char sta_pass[MAX_PASS_LEN + 1];
  char ap_ssid[MAX_SSID_LEN + 1];
  char ap_pass[MAX_PASS_LEN + 1];
} WifiCfg;

void cfg_start();
esp_err_t load_wifi_cfg(WifiCfg *cfg);
esp_err_t save_wifi_cfg(const WifiCfg *cfg);

void set_cfg_flag(cfg_flag_t flag, bool on);
bool get_cfg_flag(cfg_flag_t flag);

void set_num_zones(uint8_t num_zones);
uint8_t get_num_zones();

esp_err_t cfg_save(void);

#ifdef __cplusplus
}
#endif

#endif //AUTO_PLANT_WATERER_NVS_H