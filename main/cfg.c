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

esp_err_t load_wifi_cfg(wifi_cfg_t *cfg) {
  memset(cfg, 0, sizeof(*cfg));

  nvs_handle_t h;
  esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READONLY, &h);
  if (err == ESP_ERR_NVS_NOT_FOUND) {
    // Namespace not created yet; that's fine.
    ESP_LOGI(TAG, "NVS namespace not found; using defaults");
    goto defaults;
  }
  ESP_ERROR_CHECK(err);

  ESP_ERROR_CHECK(nvs_get_str_or_empty(h, KEY_STA_SSID, cfg->sta_ssid, sizeof(cfg->sta_ssid)));
  ESP_ERROR_CHECK(nvs_get_str_or_empty(h, KEY_STA_PASS, cfg->sta_pass, sizeof(cfg->sta_pass)));
  ESP_ERROR_CHECK(nvs_get_str_or_empty(h, KEY_AP_SSID,  cfg->ap_ssid,  sizeof(cfg->ap_ssid)));
  ESP_ERROR_CHECK(nvs_get_str_or_empty(h, KEY_AP_PASS,  cfg->ap_pass,  sizeof(cfg->ap_pass)));

  nvs_close(h);

defaults:
  if (cfg->ap_ssid[0] == 0) {
    uint8_t mac[6];
    esp_read_mac(mac, ESP_MAC_WIFI_STA);
    snprintf(cfg->ap_ssid, sizeof(cfg->ap_ssid) - 1, "%s-%02x%02x%02x", DEFAULT_AP_SSID, mac[3], mac[4], mac[5]);

    // strncpy(cfg->ap_ssid, DEFAULT_AP_SSID, sizeof(cfg->ap_ssid) - 1);
  }
  if (cfg->ap_pass[0] == 0) {
    strncpy(cfg->ap_pass, DEFAULT_AP_PASS, sizeof(cfg->ap_pass) - 1);
  }
  return ESP_OK;
}

esp_err_t save_wifi_cfg(const wifi_cfg_t *cfg) {
  nvs_handle_t h;
  ESP_ERROR_CHECK(nvs_open(NVS_NAMESPACE, NVS_READWRITE, &h));

  ESP_ERROR_CHECK(nvs_set_str(h, KEY_STA_SSID, cfg->sta_ssid));
  ESP_ERROR_CHECK(nvs_set_str(h, KEY_STA_PASS, cfg->sta_pass));
  ESP_ERROR_CHECK(nvs_set_str(h, KEY_AP_SSID,  cfg->ap_ssid));
  ESP_ERROR_CHECK(nvs_set_str(h, KEY_AP_PASS,  cfg->ap_pass));

  ESP_ERROR_CHECK(nvs_commit(h));
  nvs_close(h);
  return ESP_OK;
}

void cfg_start() {

  /* Zigbee examples expect NVS available (commissioning, network params, etc.) */
  esp_err_t err = nvs_flash_init();
  if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    ESP_ERROR_CHECK(nvs_flash_erase());
    ESP_ERROR_CHECK(nvs_flash_init());
  } else {
    ESP_ERROR_CHECK(err);
  }

}