//
// Created by Zane Claes on 1/21/26.
//

#include "cfg.h"

#include <esp_log.h>
#include <esp_mac.h>
#include <stdio.h>
#include <string.h>

#include "nvs.h"
#include "nvs_flash.h"

static const char *TAG = "cfg";

static uint32_t s_flags = 0;
static uint8_t s_num_zones = DEF_NUM_ZONES;

static esp_err_t nvs_get_str_or_empty(nvs_handle_t h, const char *key, char *out, size_t out_sz) {
  size_t needed = 0;
  esp_err_t err = nvs_get_str(h, key, NULL, &needed);
  if (err == ESP_ERR_NVS_NOT_FOUND) {
    out[0] = 0;
    return ESP_OK;
  }
  if (err != ESP_OK) return err;

  if (needed == 0) {
    out[0] = 0;
    return ESP_OK;
  }
  if (needed > out_sz) {
    // Truncate if somehow longer than our buffer (shouldn't happen).
    char *tmp = malloc(needed);
    if (!tmp) return ESP_ERR_NO_MEM;
    err = nvs_get_str(h, key, tmp, &needed);
    if (err == ESP_OK) {
      strncpy(out, tmp, out_sz - 1);
      out[out_sz - 1] = 0;
    }
    free(tmp);
    return err;
  }

  return nvs_get_str(h, key, out, &needed);
}

void set_cfg_flag(cfg_flag_t flag, bool on) {
  if (on) s_flags |= flag;
  else    s_flags &= ~flag;
}

bool get_cfg_flag(cfg_flag_t flag) { return (s_flags & flag) != 0; }
void set_num_zones(uint8_t num_zones) { s_num_zones = num_zones; }
uint8_t get_num_zones() { return s_num_zones < 1 ? DEF_NUM_ZONES : s_num_zones; }

void cfg_set(nvs_handle_t h) {
  esp_err_t err = nvs_set_u32(h, KEY_FLAGS, s_flags);
  if (err != ESP_OK) ESP_LOGW(TAG, "nvs_set: %s: %d", KEY_FLAGS, err);
  err = nvs_set_u8(h, KEY_NUM_ZONES, s_num_zones);
  if (err != ESP_OK) ESP_LOGW(TAG, "nvs_set: %s: %d", KEY_NUM_ZONES, err);
}

esp_err_t cfg_save(void) {
  nvs_handle_t h;
  esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &h);
  if (err != ESP_OK) return err;
  cfg_set(h);
  err = nvs_commit(h);
  nvs_close(h);
  return err;
}

esp_err_t load_wifi_cfg(WifiCfg *cfg) {
  memset(cfg, 0, sizeof(*cfg));

  nvs_handle_t h;
  esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READONLY, &h);
  if (err == ESP_ERR_NVS_NOT_FOUND) {
    ESP_LOGI(TAG, "NVS namespace not found; using defaults");
    goto defaults;
  }
  ESP_ERROR_CHECK(err);

  ESP_ERROR_CHECK(nvs_get_str_or_empty(h, KEY_STA_SSID, cfg->sta_ssid, sizeof(cfg->sta_ssid)));
  ESP_ERROR_CHECK(nvs_get_str_or_empty(h, KEY_STA_PASS, cfg->sta_pass, sizeof(cfg->sta_pass)));
  ESP_ERROR_CHECK(nvs_get_str_or_empty(h, KEY_AP_SSID, cfg->ap_ssid, sizeof(cfg->ap_ssid)));
  ESP_ERROR_CHECK(nvs_get_str_or_empty(h, KEY_AP_PASS, cfg->ap_pass, sizeof(cfg->ap_pass)));

  nvs_close(h);

defaults:
  if (cfg->ap_ssid[0] == 0) {
    uint8_t mac[6];
    esp_read_mac(mac, ESP_MAC_WIFI_STA);
    snprintf(cfg->ap_ssid, sizeof(cfg->ap_ssid) - 1, "%s-%02x%02x%02x", DEFAULT_AP_SSID, mac[3], mac[4], mac[5]);
  }
  if (cfg->ap_pass[0] == 0) {
    strncpy(cfg->ap_pass, DEFAULT_AP_PASS, sizeof(cfg->ap_pass) - 1);
  }
  return ESP_OK;
}

esp_err_t save_wifi_cfg(const WifiCfg *cfg) {
  nvs_handle_t h;
  ESP_ERROR_CHECK(nvs_open(NVS_NAMESPACE, NVS_READWRITE, &h));

  ESP_ERROR_CHECK(nvs_set_str(h, KEY_STA_SSID, cfg->sta_ssid));
  ESP_ERROR_CHECK(nvs_set_str(h, KEY_STA_PASS, cfg->sta_pass));
  ESP_ERROR_CHECK(nvs_set_str(h, KEY_AP_SSID, cfg->ap_ssid));
  ESP_ERROR_CHECK(nvs_set_str(h, KEY_AP_PASS, cfg->ap_pass));

  ESP_ERROR_CHECK(nvs_commit(h));
  nvs_close(h);
  return ESP_OK;
}

void cfg_start() {
  ESP_LOGI(TAG, "Cfg Starting...");
  esp_err_t err = nvs_flash_init();
  if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    ESP_ERROR_CHECK(nvs_flash_erase());
    ESP_ERROR_CHECK(nvs_flash_init());
  } else {
    ESP_ERROR_CHECK(err);
  }

  nvs_handle_t h;
  ESP_ERROR_CHECK(nvs_open(NVS_NAMESPACE, NVS_READWRITE, &h));
  err = nvs_get_u8(h, KEY_NUM_ZONES, &s_num_zones);
  if (err == ESP_ERR_NVS_NOT_FOUND) {
    ESP_LOGI(TAG, "NVS namespace creating (setting defaults)...");
    s_num_zones = DEF_NUM_ZONES;
    cfg_set(h);
    ESP_ERROR_CHECK(nvs_commit(h));
  } else { // load the rest of the settings... but still set defaults on error
    ESP_ERROR_CHECK(err);
    if (nvs_get_u32(h, KEY_FLAGS, &s_flags) != ESP_OK) { s_flags = 0; }
  }

  if (DO_RESET) {
    set_cfg_flag(CFG_FLAG_RESET, true);
    ESP_LOGW(TAG, "Build includes DO_RESET flag; will always reset!");
  }

  nvs_close(h);
}
