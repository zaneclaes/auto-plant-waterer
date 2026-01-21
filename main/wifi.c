
#include "cfg.h"
#include "wifi.h"
#include <stdio.h>
#include <string.h>
#include <lwip/ip4_addr.h>

#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/task.h"

#include "esp_event.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_netif.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_timer.h"

#include "esp_http_server.h"
#include "mdns.h"

static const char *TAG = "wifi_cfg";

// --- Event group bits ---
static EventGroupHandle_t s_wifi_evgroup;
static const int WIFI_STA_CONNECTED_BIT = BIT0;
static const int WIFI_STA_FAILED_BIT    = BIT1;
static const int WIFI_SCAN_DONE_BIT     = BIT2;

static int s_sta_retry_count = 0;

static wifi_cfg_t s_cfg;

static httpd_handle_t s_httpd = NULL;
static esp_netif_t *s_netif_sta = NULL;
static esp_netif_t *s_netif_ap  = NULL;

static void json_escape(char *dst, size_t dst_sz, const char *src) {
  if (dst_sz == 0) return;
  size_t di = 0;

  for (size_t si = 0; src && src[si] != 0; si++) {
    char c = src[si];
    // worst case we may add 2 chars (e.g. \" or \\)
    if (di + 2 >= dst_sz) break;

    if (c == '\\' || c == '\"') {
      dst[di++] = '\\';
      dst[di++] = c;
    } else if (c == '\n') {
      dst[di++] = '\\';
      dst[di++] = 'n';
    } else if (c == '\r') {
      dst[di++] = '\\';
      dst[di++] = 'r';
    } else if (c == '\t') {
      dst[di++] = '\\';
      dst[di++] = 't';
    } else {
      dst[di++] = c;
    }
  }

  dst[di] = 0;
}


// --------- WIFI scanning -----------------
#define SCAN_MAX_APS  20
#define SCAN_TIMEOUT_MS 5000

static int ssid_exists(char ssids[][MAX_SSID_LEN + 1], int count, const char *ssid) {
  for (int i = 0; i < count; i++) {
    if (strcmp(ssids[i], ssid) == 0) return 1;
  }
  return 0;
}

static int wifi_scan_ssids(char ssids_out[][MAX_SSID_LEN + 1], int max_out) {
  if (max_out <= 0) return 0;

  // Clear "done" bit before starting a scan
  xEventGroupClearBits(s_wifi_evgroup, WIFI_SCAN_DONE_BIT);

  wifi_scan_config_t scan_cfg = {
    .ssid = NULL,
    .bssid = NULL,
    .channel = 0,
    .show_hidden = false,
    .scan_type = WIFI_SCAN_TYPE_ACTIVE,
  };

  esp_err_t err = esp_wifi_scan_start(&scan_cfg, false /* non-blocking */);
  if (err != ESP_OK) {
    ESP_LOGW(TAG, "scan_start failed: %s", esp_err_to_name(err));
    return 0;
  }

  // Wait for SCAN_DONE with timeout
  EventBits_t bits = xEventGroupWaitBits(
      s_wifi_evgroup,
      WIFI_SCAN_DONE_BIT,
      pdTRUE,
      pdFALSE,
      pdMS_TO_TICKS(SCAN_TIMEOUT_MS));

  if ((bits & WIFI_SCAN_DONE_BIT) == 0) {
    ESP_LOGW(TAG, "scan timeout; stopping scan");
    esp_wifi_scan_stop();  // best-effort
    return 0;
  }

  uint16_t ap_num = SCAN_MAX_APS;
  wifi_ap_record_t ap_recs[SCAN_MAX_APS];
  memset(ap_recs, 0, sizeof(ap_recs));

  err = esp_wifi_scan_get_ap_records(&ap_num, ap_recs);
  if (err != ESP_OK) {
    ESP_LOGW(TAG, "scan_get_ap_records failed: %s", esp_err_to_name(err));
    return 0;
  }

  // Dedup
  int out = 0;
  for (int i = 0; i < ap_num && out < max_out; i++) {
    const char *ssid = (const char *)ap_recs[i].ssid;
    if (!ssid || ssid[0] == 0) continue;
    if (ssid_exists(ssids_out, out, ssid)) continue;

    strncpy(ssids_out[out], ssid, MAX_SSID_LEN);
    ssids_out[out][MAX_SSID_LEN] = 0;
    out++;
  }

  return out;
}

// -------------- Tiny URL decoding/parsing --------------

static int hexval(char c) {
  if (c >= '0' && c <= '9') return c - '0';
  if (c >= 'a' && c <= 'f') return 10 + (c - 'a');
  if (c >= 'A' && c <= 'F') return 10 + (c - 'A');
  return -1;
}

// Decodes application/x-www-form-urlencoded into dst (in-place safe if dst==src)
static void url_decode(char *dst, const char *src, size_t dst_sz) {
  size_t di = 0;
  for (size_t si = 0; src[si] && di + 1 < dst_sz; si++) {
    if (src[si] == '+') {
      dst[di++] = ' ';
    } else if (src[si] == '%' && src[si+1] && src[si+2]) {
      int hi = hexval(src[si+1]);
      int lo = hexval(src[si+2]);
      if (hi >= 0 && lo >= 0) {
        dst[di++] = (char)((hi << 4) | lo);
        si += 2;
      } else {
        dst[di++] = src[si];
      }
    } else {
      dst[di++] = src[si];
    }
  }
  dst[di] = 0;
}

static void form_get_value(const char *body, const char *key, char *out, size_t out_sz) {
  out[0] = 0;
  size_t keylen = strlen(key);

  const char *p = body;
  while (p && *p) {
    const char *eq = strchr(p, '=');
    if (!eq) break;
    const char *amp = strchr(p, '&');
    size_t klen = (size_t)(eq - p);

    if (klen == keylen && strncmp(p, key, keylen) == 0) {
      const char *vstart = eq + 1;
      size_t vlen = amp ? (size_t)(amp - vstart) : strlen(vstart);

      // Copy raw then decode
      size_t copy = (vlen < out_sz - 1) ? vlen : (out_sz - 1);
      memcpy(out, vstart, copy);
      out[copy] = 0;

      char decoded[256];
      url_decode(decoded, out, sizeof(decoded));
      strncpy(out, decoded, out_sz - 1);
      out[out_sz - 1] = 0;
      return;
    }

    p = amp ? (amp + 1) : NULL;
  }
}

// ---------------- HTTP server ----------------
extern const uint8_t index_html_start[] asm("_binary_index_html_start");
extern const uint8_t index_html_end[]   asm("_binary_index_html_end");

static esp_err_t http_get_root(httpd_req_t *req)
{
  const char *data = (const char *)index_html_start;
  const size_t size = (size_t)(index_html_end - index_html_start);

  httpd_resp_set_type(req, "text/html");
  httpd_resp_set_hdr(req, "Cache-Control", "no-store");

  // Send in chunks to avoid EAGAIN on big sends
  const size_t CHUNK = 1024;
  size_t off = 0;

  while (off < size) {
    size_t n = size - off;
    if (n > CHUNK) n = CHUNK;

    esp_err_t err = httpd_resp_send_chunk(req, data + off, n);
    if (err != ESP_OK) {
      // Most commonly you'll see failures here if the client disconnects or socket would block
      ESP_LOGW(TAG, "send_chunk failed at off=%u/%u: %s",
               (unsigned)off, (unsigned)size, esp_err_to_name(err));
      // Tell server we're done (best effort) and fail
      httpd_resp_sendstr_chunk(req, NULL);
      return err;
    }

    off += n;
  }

  // Finalize chunked response
  return httpd_resp_sendstr_chunk(req, NULL);
}


static esp_err_t http_get_config(httpd_req_t *req) {
  // Escape strings for JSON
  char sta_ssid[96], sta_pass[96], ap_ssid[96], ap_pass[96];
  json_escape(sta_ssid, sizeof(sta_ssid), s_cfg.sta_ssid);
  json_escape(sta_pass, sizeof(sta_pass), s_cfg.sta_pass);
  json_escape(ap_ssid,  sizeof(ap_ssid),  s_cfg.ap_ssid);
  json_escape(ap_pass,  sizeof(ap_pass),  s_cfg.ap_pass);

  // Small fixed response; keep it simple and safe
  char json[512];
  int n = snprintf(
      json, sizeof(json),
      "{\"sta_ssid\":\"%s\",\"sta_pass\":\"%s\",\"ap_ssid\":\"%s\",\"ap_pass\":\"%s\"}",
      sta_ssid, sta_pass, ap_ssid, ap_pass);

  if (n < 0 || n >= (int)sizeof(json)) {
    httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "config too large");
    return ESP_FAIL;
  }

  httpd_resp_set_type(req, "application/json");
  httpd_resp_set_hdr(req, "Cache-Control", "no-store");
  return httpd_resp_send(req, json, HTTPD_RESP_USE_STRLEN);
}


static esp_err_t http_get_scan(httpd_req_t *req) {
  ESP_LOGI(TAG, "HTTP /scan");

  char ssids[SCAN_MAX_APS][MAX_SSID_LEN + 1];
  memset(ssids, 0, sizeof(ssids));
  int count = wifi_scan_ssids(ssids, SCAN_MAX_APS);

  // worst-case sizing: {"ssids":[...]} + quotes/commas
  //  (32 chars + 3) * SCAN_MAX_APS is safe-ish, plus header
  size_t cap = 32 + (size_t)count * (MAX_SSID_LEN + 4) + 16;
  char *json = (char *)malloc(cap);
  if (!json) {
    httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "OOM");
    return ESP_FAIL;
  }

  int n = 0;
  n += snprintf(json + n, cap - (size_t)n, "{\"ssids\":[");
  for (int i = 0; i < count && n < (int)cap - 8; i++) {
    n += snprintf(json + n, cap - (size_t)n, "%s\"%s\"", (i ? "," : ""), ssids[i]);
  }
  n += snprintf(json + n, cap - (size_t)n, "]}");

  httpd_resp_set_type(req, "application/json");
  httpd_resp_set_hdr(req, "Cache-Control", "no-store");
  esp_err_t err = httpd_resp_send(req, json, HTTPD_RESP_USE_STRLEN);
  free(json);
  return err;
}

static void wifi_restart_task(void *arg);

static esp_err_t http_post_save(httpd_req_t *req) {
  // Read body
  int total = req->content_len;
  if (total <= 0 || total > 1024) {
    httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Bad content length");
    return ESP_FAIL;
  }

  char *buf = malloc((size_t)total + 1);
  if (!buf) {
    httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "OOM");
    return ESP_FAIL;
  }

  int got = 0;
  while (got < total) {
    int r = httpd_req_recv(req, buf + got, total - got);
    if (r <= 0) { free(buf); httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Recv failed"); return ESP_FAIL; }
    got += r;
  }
  buf[total] = 0;

  // Parse form values into temp cfg
  wifi_cfg_t new_cfg = s_cfg;
  form_get_value(buf, "sta_ssid", new_cfg.sta_ssid, sizeof(new_cfg.sta_ssid));
  form_get_value(buf, "sta_pass", new_cfg.sta_pass, sizeof(new_cfg.sta_pass));
  form_get_value(buf, "ap_ssid",  new_cfg.ap_ssid,  sizeof(new_cfg.ap_ssid));
  form_get_value(buf, "ap_pass",  new_cfg.ap_pass,  sizeof(new_cfg.ap_pass));

  free(buf);

  // Apply defaults if AP SSID empty
  if (new_cfg.ap_ssid[0] == 0) {
    strncpy(new_cfg.ap_ssid, DEFAULT_AP_SSID, sizeof(new_cfg.ap_ssid) - 1);
  }

  // WPA2 rule: if password length is 1..7 it's invalid; treat as open or ask user
  size_t aplen = strlen(new_cfg.ap_pass);
  if (aplen > 0 && aplen < 8) {
    httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "AP password must be >= 8 chars or blank");
    return ESP_FAIL;
  }

  ESP_ERROR_CHECK(save_wifi_cfg(&new_cfg));
  s_cfg = new_cfg;

  httpd_resp_set_type(req, "text/html");
  httpd_resp_sendstr(req, "<html><body><h3>Saved.</h3><p>Restarting Wi-Fi…</p></body></html>");

  // Restart Wi-Fi asynchronously so we can finish the HTTP response.
  xTaskCreate(wifi_restart_task, "wifi_restart", 4096, NULL, 5, NULL);
  return ESP_OK;
}

static httpd_handle_t start_http_server(void) {
  httpd_config_t cfg = HTTPD_DEFAULT_CONFIG();
  cfg.stack_size = 4096;
  cfg.max_uri_handlers = 8;

  httpd_handle_t server = NULL;
  if (httpd_start(&server, &cfg) != ESP_OK) {
    ESP_LOGE(TAG, "Failed to start HTTP server");
    return NULL;
  }

  httpd_uri_t uri_root = {
    .uri = "/",
    .method = HTTP_GET,
    .handler = http_get_root,
    .user_ctx = NULL
  };
  httpd_register_uri_handler(server, &uri_root);

  httpd_uri_t uri_scan = {
    .uri = "/scan",
    .method = HTTP_GET,
    .handler = http_get_scan,
    .user_ctx = NULL
  };
  httpd_register_uri_handler(server, &uri_scan);

  httpd_uri_t uri_config = {
    .uri = "/config",
    .method = HTTP_GET,
    .handler = http_get_config,
    .user_ctx = NULL
  };
  httpd_register_uri_handler(server, &uri_config);

  httpd_uri_t uri_save = {
    .uri = "/save",
    .method = HTTP_POST,
    .handler = http_post_save,
    .user_ctx = NULL
  };
  httpd_register_uri_handler(server, &uri_save);

  return server;
}

// ---------------- mDNS ----------------

static void start_mdns(const char *hostname) {
  ESP_ERROR_CHECK(mdns_init());
  ESP_ERROR_CHECK(mdns_hostname_set(hostname));
  ESP_ERROR_CHECK(mdns_instance_name_set("Planter Device"));
  // Advertise HTTP
  ESP_ERROR_CHECK(mdns_service_add(NULL, "_http", "_tcp", 80, NULL, 0));
  ESP_LOGI(TAG, "mDNS started: http://%s.local/", hostname);
}

// ---------------- Wi-Fi event handler ----------------

static void wifi_event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data) {
  if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
    ESP_LOGI(TAG, "STA start -> connecting...");
    esp_wifi_connect();
  }
  else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
    const wifi_event_sta_disconnected_t *disc = (const wifi_event_sta_disconnected_t *)event_data;
    ESP_LOGW(TAG, "STA disconnected reason=%d", disc->reason);

    if (s_sta_retry_count < STA_MAX_RETRIES) {
      s_sta_retry_count++;
      esp_wifi_connect();
      ESP_LOGI(TAG, "Retrying STA... (%d/%d)", s_sta_retry_count, STA_MAX_RETRIES);
    } else {
      xEventGroupSetBits(s_wifi_evgroup, WIFI_STA_FAILED_BIT);
    }
  }
  else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
    ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
    ESP_LOGI(TAG, "Got IP: " IPSTR, IP2STR(&event->ip_info.ip));
    s_sta_retry_count = 0;
    xEventGroupSetBits(s_wifi_evgroup, WIFI_STA_CONNECTED_BIT);
  }
  else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_SCAN_DONE) {
    xEventGroupSetBits(s_wifi_evgroup, WIFI_SCAN_DONE_BIT);
  }
}

// ---------------- Wi-Fi start modes ----------------

static void wifi_init_common(void) {
  s_wifi_evgroup = xEventGroupCreate();

  ESP_ERROR_CHECK(esp_netif_init());
  ESP_ERROR_CHECK(esp_event_loop_create_default());

  // Create both netifs up front (AP + STA)
  s_netif_sta = esp_netif_create_default_wifi_sta();
  s_netif_ap  = esp_netif_create_default_wifi_ap();

  wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
  ESP_ERROR_CHECK(esp_wifi_init(&cfg));

  ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL));
  ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL));

  ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
}

static esp_err_t start_softap(const char *ap_ssid, const char *ap_pass) {
  wifi_config_t ap_cfg = { 0 };

  strncpy((char *)ap_cfg.ap.ssid, ap_ssid, sizeof(ap_cfg.ap.ssid) - 1);
  ap_cfg.ap.ssid_len = (uint8_t)strlen(ap_ssid);
  ap_cfg.ap.channel = DEFAULT_AP_CHANNEL;
  ap_cfg.ap.max_connection = DEFAULT_AP_MAX_CONN;

  if (ap_pass && ap_pass[0] != 0) {
    strncpy((char *)ap_cfg.ap.password, ap_pass, sizeof(ap_cfg.ap.password) - 1);
    ap_cfg.ap.authmode = WIFI_AUTH_WPA2_PSK;
  } else {
    ap_cfg.ap.password[0] = 0;
    ap_cfg.ap.authmode = WIFI_AUTH_OPEN;
  }

  ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_APSTA));
  ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &ap_cfg));
  ESP_ERROR_CHECK(esp_wifi_start());

  ESP_LOGI(TAG, "SoftAP started. SSID='%s' auth=%s",
           ap_ssid, (ap_cfg.ap.authmode == WIFI_AUTH_OPEN) ? "OPEN" : "WPA2");
  return ESP_OK;
}

static esp_err_t try_start_station(const char *sta_ssid, const char *sta_pass) {
  if (!sta_ssid || sta_ssid[0] == 0) return ESP_FAIL;

  wifi_config_t sta_cfg = { 0 };
  strncpy((char *)sta_cfg.sta.ssid, sta_ssid, sizeof(sta_cfg.sta.ssid) - 1);
  if (sta_pass) {
    strncpy((char *)sta_cfg.sta.password, sta_pass, sizeof(sta_cfg.sta.password) - 1);
  }

  // Optional: improve roaming behavior
  sta_cfg.sta.scan_method = WIFI_FAST_SCAN;
  sta_cfg.sta.sort_method = WIFI_CONNECT_AP_BY_SIGNAL;

  ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
  ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &sta_cfg));
  ESP_ERROR_CHECK(esp_wifi_start());

  ESP_LOGI(TAG, "Attempting STA connect to SSID='%s'...", sta_ssid);

  EventBits_t bits = xEventGroupWaitBits(
      s_wifi_evgroup,
      WIFI_STA_CONNECTED_BIT | WIFI_STA_FAILED_BIT,
      pdTRUE,
      pdFALSE,
      pdMS_TO_TICKS(STA_CONNECT_TIMEOUT_MS));

  if (bits & WIFI_STA_CONNECTED_BIT) {
    ESP_LOGI(TAG, "STA connected.");
    return ESP_OK;
  }

  ESP_LOGW(TAG, "STA connect failed/timeout; stopping STA");
  esp_wifi_stop();
  vTaskDelay(pdMS_TO_TICKS(250));
  return ESP_FAIL;
}

// Restart Wi-Fi after saving new settings (called from HTTP)
static void wifi_restart_task(void *arg) {
  (void)arg;

  ESP_LOGI(TAG, "Restarting Wi-Fi with new config...");
  esp_wifi_stop();
  vTaskDelay(pdMS_TO_TICKS(300));

  // Try STA first if configured, else AP
  if (s_cfg.sta_ssid[0] != 0 && try_start_station(s_cfg.sta_ssid, s_cfg.sta_pass) == ESP_OK) {
    // good
  } else {
    start_softap(s_cfg.ap_ssid, s_cfg.ap_pass);
  }

  vTaskDelete(NULL);
}

// Pick an available mDNS hostname by probing planter/planter2/planter3...
// Returns ESP_OK and writes chosen hostname into out (NUL-terminated).
static esp_err_t mdns_pick_hostname(int max_suffix, char *out, size_t out_sz)
{
    if (!out || out_sz < 2) return ESP_ERR_INVALID_ARG;

    // 1) Init mDNS (once)
    ESP_ERROR_CHECK(mdns_init());

    // 2) Set a temporary UNIQUE hostname to avoid "self-query" pitfalls.
    //    (Docs note self-querying isn’t supported, so we probe names before claiming them.) :contentReference[oaicite:3]{index=3}
    uint8_t mac[6];
    esp_read_mac(mac, ESP_MAC_WIFI_STA);
    char tmp[32];
    snprintf(tmp, sizeof(tmp), "%s-%02x%02x%02x", DEFAULT_AP_SSID, mac[3], mac[4], mac[5]);
    ESP_ERROR_CHECK(mdns_hostname_set(tmp));

    // 3) Probe candidates on the LAN via A-record queries. :contentReference[oaicite:4]{index=4}
    for (int i = 0; i <= max_suffix; i++) {
        char candidate[64];
        if (i == 0) snprintf(candidate, sizeof(candidate), "%s", DEFAULT_AP_SSID);
        else        snprintf(candidate, sizeof(candidate), "%s%d", DEFAULT_AP_SSID, i + 1); // planter2, planter3...

        esp_ip4_addr_t addr = {0};
        esp_err_t err = mdns_query_a(candidate, 500 /*ms*/, &addr);

        if (err == ESP_ERR_NOT_FOUND) {
            // No one answered -> likely free, claim it.
            ESP_ERROR_CHECK(mdns_hostname_set(candidate));

            // Optional: read back what mDNS thinks the hostname is. :contentReference[oaicite:5]{index=5}
            char confirmed[64] = {0};
            if (mdns_hostname_get(confirmed) == ESP_OK) {
                ESP_LOGI(TAG, "mDNS hostname set: %s.local", confirmed);
            } else {
                ESP_LOGI(TAG, "mDNS hostname set: %s.local", candidate);
            }

            strlcpy(out, candidate, out_sz);
            return ESP_OK;
        }

        // If err == ESP_OK, someone responded => taken.
        // Other errors (invalid state, etc.) you may want to treat as "retry".
        if (err == ESP_OK) {
            ESP_LOGI(TAG, "Hostname taken: %s.local -> " IPSTR, candidate, IP2STR((ip4_addr_t*)&addr));
        } else {
            ESP_LOGW(TAG, "Probe error for %s: %s", candidate, esp_err_to_name(err));
        }
    }

    return ESP_ERR_NOT_FOUND; // none available within max_suffix
}

// ---------------- App entry ----------------

void wifi_start(void) {
  ESP_ERROR_CHECK(load_wifi_cfg(&s_cfg));

  // Wi-Fi init
  wifi_init_common();

  // Derive a stable mDNS hostname: "device" + last 3 bytes of MAC
  char hostname[32];
  ESP_ERROR_CHECK(mdns_pick_hostname(20, hostname, sizeof(hostname)));
  start_mdns(hostname);

  // Start HTTP server (works on whichever interface is up)
  s_httpd = start_http_server();
  if (!s_httpd) {
    ESP_LOGE(TAG, "HTTP server failed to start (continuing anyway)");
  } else {
    ESP_LOGI(TAG, "HTTP config page: http://%s.local/  (or device IP)", hostname);
  }

  // Try station if configured; fallback to softAP
  if (s_cfg.sta_ssid[0] != 0 && try_start_station(s_cfg.sta_ssid, s_cfg.sta_pass) == ESP_OK) {
    // STA connected, we’re done.
    // If you want BOTH STA + AP simultaneously, switch to WIFI_MODE_APSTA and configure both.
    ESP_LOGI(TAG, "Running on STA network.");
  } else {
    ESP_LOGI(TAG, "Starting fallback AP...");
    ESP_ERROR_CHECK(start_softap(s_cfg.ap_ssid, s_cfg.ap_pass));
    ESP_LOGI(TAG, "Connect to AP, then open http://%s.local/ or http://192.168.4.1/", hostname);
  }
}