#include <string.h>
#include <inttypes.h>
#include "zb.h"
#include "cfg.h"
#include "tof.h"
#include "pumps.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"

#include "esp_log.h"
#include "esp_err.h"
#include "driver/gpio.h"

#include "esp_zigbee_core.h"
#include "zcl/esp_zigbee_zcl_common.h"
#include "zcl/esp_zigbee_zcl_command.h"
// #include "zcl/esp_zigbee_zcl_analog_input.h"
#include "bat.h"
#include "coord.h"
#include "esp_mac.h"
#include "esp_pm.h"
#include "esp_timer.h"
#include "soil.h"
#include "zcl/esp_zigbee_zcl_humidity_meas.h"
#include "zcl/esp_zigbee_zcl_power_config.h"

static const char *TAG = "zb";

/* -----------------------------
 * USER CONFIG
 * ----------------------------- */
#define COORDINATOR_SHORT_ADDR 0x0000
#define COORDINATOR_ENDPOINT   1   // Z2M coordinator endpoint is typically 1

#define EP_WATER      0x01
#define EP_SOIL1      0x02
#define EP_SOIL2      0x03
#define EP_SOIL3      0x04

#define WATER_CLUSTER_ID              0xFF01
#define WATER_ATTR_WATER_LEVEL_MM_ID  0x0000

/* -----------------------------
 * INTERNALS
 * ----------------------------- */

static uint16_t s_immediate_water_level_change_report = 500; // if any water/soil level changes more than pct, report immediately
static uint16_t s_water_level_pct_x100 = 0; // 0..10000  (0.01% units)
static uint16_t s_rh_min_x100 = 0; // 0.00%
static uint16_t s_rh_max_x100 = 10000; // 100.00%

static uint16_t s_soil_level_pct_x100[3] = { 0, 0, 0 };

static uint8_t battery_pct_reportable_change = 2; // 1% (0.5% units)
static uint8_t battery_v_reportable_change   = 1; // 0.1V (100mV units)
static uint8_t s_battery_voltage = 0;
static uint8_t s_battery_percent = 0;

/* -----------------------------
 * Helpers
 * ----------------------------- */

static void zb_get_battery(void) {
  const struct BatteryLevel* bat = get_battery();
  s_battery_percent = bat->percent * 2;
  s_battery_voltage = (uint8_t)(bat->voltage * 10.0f);
}

static uint8_t get_soil_level_endpoint(uint8_t idx) {
  uint8_t num_zones = get_num_zones();
  if (idx == 0 && num_zones >= 1) return EP_SOIL1;
  if (idx == 1 && num_zones >= 2) return EP_SOIL2;
  if (idx == 2 && num_zones >= 3) return EP_SOIL3;
  ESP_LOGW(TAG, "Soil level index #%u invalid (%u zones)", idx, num_zones);
  return EP_SOIL1;
}

static uint8_t get_soil_level_idx(uint8_t ep) {
  if (ep == EP_SOIL1) return 0;
  if (ep == EP_SOIL2) return 1;
  if (ep == EP_SOIL3) return 2;
  ESP_LOGW(TAG, "Soil level endpoint %u invalid", ep);
  return 0;
}

/* -----------------------------
 * I2C / ToF (stub)
 * ----------------------------- */

static void water_level_bind_cb(esp_zb_zdp_status_t zdo_status, void *user_ctx) {
  esp_zb_zdo_bind_req_param_t *bind_req = (esp_zb_zdo_bind_req_param_t *) user_ctx;

  if (zdo_status == ESP_ZB_ZDP_STATUS_SUCCESS) {
    esp_zb_zcl_config_report_cmd_t report_cmd = {0};

    report_cmd.zcl_basic_cmd.dst_addr_u.addr_short = esp_zb_get_short_address(); // local config
    report_cmd.zcl_basic_cmd.src_endpoint = bind_req->src_endp;
    report_cmd.zcl_basic_cmd.dst_endpoint = COORDINATOR_ENDPOINT;
    report_cmd.address_mode = ESP_ZB_APS_ADDR_MODE_16_ENDP_PRESENT;

    report_cmd.clusterID = ESP_ZB_ZCL_CLUSTER_ID_REL_HUMIDITY_MEASUREMENT;

    esp_zb_zcl_config_report_record_t rec = {
      .direction = ESP_ZB_ZCL_REPORT_DIRECTION_SEND,
      .attributeID = ESP_ZB_ZCL_ATTR_REL_HUMIDITY_MEASUREMENT_VALUE_ID,
      .attrType = ESP_ZB_ZCL_ATTR_TYPE_U16,
      .min_interval = DEF_REPORTING_MIN_SEC,
      .max_interval = DEF_REPORTING_MAX_SEC,
      .reportable_change = &s_immediate_water_level_change_report,
    };

    report_cmd.record_number = 1;
    report_cmd.record_field = &rec;

    esp_zb_lock_acquire(portMAX_DELAY);
    esp_zb_zcl_config_report_cmd_req(&report_cmd);
    esp_zb_lock_release();

    ESP_LOGI(TAG, "Water level reporting configured (humidity from ep %d)", bind_req->src_endp);
  } else {
    ESP_LOGW(TAG, "Bind failed: zdo_status=%d", zdo_status);
  }

  free(bind_req);
}

static void water_level_bind_to_coordinator(void) {
  esp_zb_zdo_bind_req_param_t *bind_req =
      (esp_zb_zdo_bind_req_param_t *) calloc(1, sizeof(esp_zb_zdo_bind_req_param_t));

  bind_req->req_dst_addr = esp_zb_get_short_address();
  bind_req->src_endp = EP_WATER;
  bind_req->dst_endp = COORDINATOR_ENDPOINT;
  bind_req->cluster_id = ESP_ZB_ZCL_CLUSTER_ID_REL_HUMIDITY_MEASUREMENT;
  bind_req->dst_addr_mode = ESP_ZB_ZDO_BIND_DST_ADDR_MODE_64_BIT_EXTENDED;

  esp_zb_ieee_address_by_short(COORDINATOR_SHORT_ADDR, bind_req->dst_address_u.addr_long);
  esp_zb_get_long_address(bind_req->src_address);
  esp_zb_zdo_device_bind_req(bind_req, water_level_bind_cb, bind_req);
}

static void soil_level_bind_cb(esp_zb_zdp_status_t zdo_status, void *user_ctx) {
  esp_zb_zdo_bind_req_param_t *bind_req = (esp_zb_zdo_bind_req_param_t *) user_ctx;

  if (zdo_status == ESP_ZB_ZDP_STATUS_SUCCESS) {
    esp_zb_zcl_config_report_cmd_t report_cmd = {0};

    report_cmd.zcl_basic_cmd.dst_addr_u.addr_short = esp_zb_get_short_address();
    report_cmd.zcl_basic_cmd.src_endpoint = bind_req->src_endp;
    report_cmd.zcl_basic_cmd.dst_endpoint = COORDINATOR_ENDPOINT;
    report_cmd.address_mode = ESP_ZB_APS_ADDR_MODE_16_ENDP_PRESENT;

    report_cmd.clusterID = ESP_ZB_ZCL_CLUSTER_ID_REL_HUMIDITY_MEASUREMENT;

    esp_zb_zcl_config_report_record_t rec = {
      .direction = ESP_ZB_ZCL_REPORT_DIRECTION_SEND,
      .attributeID = ESP_ZB_ZCL_ATTR_REL_HUMIDITY_MEASUREMENT_VALUE_ID,
      .attrType = ESP_ZB_ZCL_ATTR_TYPE_U16,
      .min_interval = DEF_REPORTING_MIN_SEC,
      .max_interval = DEF_REPORTING_MAX_SEC,
      .reportable_change = &s_immediate_water_level_change_report,
    };

    report_cmd.record_number = 1;
    report_cmd.record_field = &rec;

    esp_zb_lock_acquire(portMAX_DELAY);
    esp_zb_zcl_config_report_cmd_req(&report_cmd);
    esp_zb_lock_release();

    ESP_LOGI(TAG, "Soil level reporting configured (humidity from ep %d)", bind_req->src_endp);
  } else {
    ESP_LOGW(TAG, "Bind failed: zdo_status=%d", zdo_status);
  }

  free(bind_req);
}

static void soil_level_bind_to_coordinator(uint8_t idx) {
  esp_zb_zdo_bind_req_param_t *bind_req =
      (esp_zb_zdo_bind_req_param_t *) calloc(1, sizeof(esp_zb_zdo_bind_req_param_t));

  bind_req->req_dst_addr = esp_zb_get_short_address();
  bind_req->src_endp = get_soil_level_endpoint(idx);
  bind_req->dst_endp = COORDINATOR_ENDPOINT;
  bind_req->cluster_id = ESP_ZB_ZCL_CLUSTER_ID_REL_HUMIDITY_MEASUREMENT;
  bind_req->dst_addr_mode = ESP_ZB_ZDO_BIND_DST_ADDR_MODE_64_BIT_EXTENDED;

  esp_zb_ieee_address_by_short(COORDINATOR_SHORT_ADDR, bind_req->dst_address_u.addr_long);
  esp_zb_get_long_address(bind_req->src_address);
  esp_zb_zdo_device_bind_req(bind_req, soil_level_bind_cb, bind_req);
}

static void bind_battery_cb(esp_zb_zdp_status_t zdo_status, void *user_ctx){
  esp_zb_zdo_bind_req_param_t *bind_req = (esp_zb_zdo_bind_req_param_t *)user_ctx;

  if (zdo_status == ESP_ZB_ZDP_STATUS_SUCCESS) {
    esp_zb_zcl_config_report_cmd_t cmd = {0};

    cmd.address_mode = ESP_ZB_APS_ADDR_MODE_16_ENDP_PRESENT;
    cmd.zcl_basic_cmd.dst_addr_u.addr_short = esp_zb_get_short_address();
    cmd.zcl_basic_cmd.src_endpoint = bind_req->src_endp;
    cmd.zcl_basic_cmd.dst_endpoint = COORDINATOR_ENDPOINT;
    cmd.clusterID = ESP_ZB_ZCL_CLUSTER_ID_POWER_CONFIG;

    esp_zb_zcl_config_report_record_t recs[2] = {
      {
        .direction = ESP_ZB_ZCL_REPORT_DIRECTION_SEND,
        .attributeID = ESP_ZB_ZCL_ATTR_POWER_CONFIG_BATTERY_PERCENTAGE_REMAINING_ID,
        .attrType = ESP_ZB_ZCL_ATTR_TYPE_U8,
        .min_interval = BAT_REPORTING_MIN_SEC,
        .max_interval = BAT_REPORTING_MAX_SEC,
        .reportable_change = &battery_pct_reportable_change,
    },
    {
      .direction = ESP_ZB_ZCL_REPORT_DIRECTION_SEND,
      .attributeID = ESP_ZB_ZCL_ATTR_POWER_CONFIG_BATTERY_VOLTAGE_ID,
      .attrType = ESP_ZB_ZCL_ATTR_TYPE_U8,
      .min_interval = BAT_REPORTING_MIN_SEC,
      .max_interval = BAT_REPORTING_MAX_SEC,
      .reportable_change = &battery_v_reportable_change,
    }
    };

    cmd.record_number = 2;
    cmd.record_field  = recs;

    esp_zb_lock_acquire(portMAX_DELAY);
    esp_zb_zcl_config_report_cmd_req(&cmd);
    esp_zb_lock_release();

    ESP_LOGI(TAG, "Bind OK: battery reports from ep %d to coordinator ep %d",
             bind_req->src_endp, bind_req->dst_endp);
  } else {
    ESP_LOGW(TAG, "Bind failed: zdo_status=%d", zdo_status);
  }

  free(bind_req);
}

static void battery_bind_to_coordinator() {
  esp_zb_zdo_bind_req_param_t *bind_req =
      calloc(1, sizeof(esp_zb_zdo_bind_req_param_t));

  bind_req->req_dst_addr = esp_zb_get_short_address(); // local device
  bind_req->src_endp     = EP_WATER;
  bind_req->dst_endp =    COORDINATOR_ENDPOINT;
  bind_req->cluster_id   = ESP_ZB_ZCL_CLUSTER_ID_POWER_CONFIG;
  bind_req->dst_addr_mode = ESP_ZB_ZDO_BIND_DST_ADDR_MODE_64_BIT_EXTENDED;

  esp_zb_get_long_address(bind_req->src_address);
  esp_zb_ieee_address_by_short(0x0000, bind_req->dst_address_u.addr_long);

  esp_zb_zdo_device_bind_req(bind_req, bind_battery_cb, bind_req);
}

static void on_zb_joined() {
  if (is_joined()) {
    ESP_LOGI(TAG, "double-join (skipping re-bind)");
    return;
  }
  ESP_LOGI(TAG, "Joined network OK (PAN: 0x%04hx, short: 0x%04hx, MAC: %s)",
           esp_zb_get_pan_id(), esp_zb_get_short_address(), get_mac_addr());
  on_joined();
  water_level_bind_to_coordinator();
  battery_bind_to_coordinator();
  uint8_t num_zones = get_num_zones();
  for (uint8_t i = 0; i < num_zones; i++) {
    soil_level_bind_to_coordinator(i);
  }
  zb_report_battery();
}

void zb_on_ready() {
  ESP_LOGI(TAG, "Enabling sleep...");
  // esp_zb_sleep_set_threshold(200);
  // esp_zb_sleep_enable(true);
  // esp_zb_set_rx_on_when_idle(false);
}

/* -----------------------------
 * Zigbee: device model creation
 * ----------------------------- */

static esp_zb_cluster_list_t *create_water_level_clusters(uint8_t ep) {
  esp_zb_cluster_list_t *cluster_list = esp_zb_zcl_cluster_list_create();

  if (ep == EP_WATER) {
    esp_zb_basic_cluster_cfg_t basic_cfg = {0};
    esp_zb_attribute_list_t *basic = esp_zb_basic_cluster_create(&basic_cfg);
    esp_zb_basic_cluster_add_attr(basic, ESP_ZB_ZCL_ATTR_BASIC_MANUFACTURER_NAME_ID, (void *) ZB_MANUFACTURER_NAME);
    esp_zb_basic_cluster_add_attr(basic, ESP_ZB_ZCL_ATTR_BASIC_MODEL_IDENTIFIER_ID, (void *) ZB_MODEL_IDENTIFIER);
    esp_zb_basic_cluster_add_attr(basic, ESP_ZB_ZCL_ATTR_BASIC_SW_BUILD_ID, ZB_VERSION_NUMBER);
    esp_zb_cluster_list_add_basic_cluster(cluster_list, basic, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);
  }

  esp_zb_identify_cluster_cfg_t identify_cfg = {0};
  esp_zb_attribute_list_t *identify = esp_zb_identify_cluster_create(&identify_cfg);
  esp_zb_cluster_list_add_identify_cluster(cluster_list, identify, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);

  esp_zb_attribute_list_t *rh =
      esp_zb_zcl_attr_list_create(ESP_ZB_ZCL_CLUSTER_ID_REL_HUMIDITY_MEASUREMENT);

  if (ep == EP_WATER) {
    esp_zb_custom_cluster_add_custom_attr(
      rh,
      ESP_ZB_ZCL_ATTR_REL_HUMIDITY_MEASUREMENT_VALUE_ID,
      ESP_ZB_ZCL_ATTR_TYPE_U16,
      ESP_ZB_ZCL_ATTR_ACCESS_READ_ONLY | ESP_ZB_ZCL_ATTR_ACCESS_REPORTING,
      &s_water_level_pct_x100
    );
  } else {
    esp_zb_custom_cluster_add_custom_attr(
      rh,
      ESP_ZB_ZCL_ATTR_REL_HUMIDITY_MEASUREMENT_VALUE_ID,
      ESP_ZB_ZCL_ATTR_TYPE_U16,
      ESP_ZB_ZCL_ATTR_ACCESS_READ_ONLY | ESP_ZB_ZCL_ATTR_ACCESS_REPORTING,
      &s_soil_level_pct_x100[get_soil_level_idx(ep)]
    );
  }

  esp_zb_custom_cluster_add_custom_attr(
    rh,
    ESP_ZB_ZCL_ATTR_REL_HUMIDITY_MEASUREMENT_MIN_VALUE_ID,
    ESP_ZB_ZCL_ATTR_TYPE_U16,
    ESP_ZB_ZCL_ATTR_ACCESS_READ_ONLY,
    &s_rh_min_x100
  );

  esp_zb_custom_cluster_add_custom_attr(
    rh,
    ESP_ZB_ZCL_ATTR_REL_HUMIDITY_MEASUREMENT_MAX_VALUE_ID,
    ESP_ZB_ZCL_ATTR_TYPE_U16,
    ESP_ZB_ZCL_ATTR_ACCESS_READ_ONLY,
    &s_rh_max_x100
  );

  esp_zb_cluster_list_add_humidity_meas_cluster(cluster_list, rh, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);

  if (ep == EP_WATER) {
    zb_get_battery();
    // esp_zb_attribute_list_t *pwr =
    //   esp_zb_zcl_attr_list_create(ESP_ZB_ZCL_CLUSTER_ID_POWER_CONFIG);
    //
    esp_zb_power_config_cluster_cfg_t pwr_cfg = {0};
    esp_zb_attribute_list_t *pwr = esp_zb_power_config_cluster_create(&pwr_cfg);

    // Then add just the attributes you actually support.
    esp_zb_custom_cluster_add_custom_attr(
        pwr,
        ESP_ZB_ZCL_ATTR_POWER_CONFIG_BATTERY_VOLTAGE_ID,
        ESP_ZB_ZCL_ATTR_TYPE_U8,
        ESP_ZB_ZCL_ATTR_ACCESS_READ_ONLY | ESP_ZB_ZCL_ATTR_ACCESS_REPORTING,
        &s_battery_voltage
    );

    esp_zb_custom_cluster_add_custom_attr(
        pwr,
        ESP_ZB_ZCL_ATTR_POWER_CONFIG_BATTERY_PERCENTAGE_REMAINING_ID,
        ESP_ZB_ZCL_ATTR_TYPE_U8,
        ESP_ZB_ZCL_ATTR_ACCESS_READ_ONLY | ESP_ZB_ZCL_ATTR_ACCESS_REPORTING,
        &s_battery_percent
    );

    esp_zb_cluster_list_add_power_config_cluster(cluster_list, pwr, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);
  }

  return cluster_list;
}

/* -----------------------------
 * Zigbee: signal handler (commissioning / join)
* ----------------------------- */
static bool zb_check_joined() {
  if (esp_zb_bdb_is_factory_new()) return false;
  uint16_t short_addr = esp_zb_get_short_address();
  if (short_addr != 0xFFFF && short_addr != 0xFFFE && short_addr != 0x0000) {
    on_zb_joined();
    return true;
  }
  return false;
}

static void zb_join(uint8_t param) {
  if (is_joined()) {
    ESP_LOGI(TAG, "Skipping join (already joined)");
    return;
  }
  if (is_joining()) {
    ESP_LOGI(TAG, "Skipping join (already trying to join)");
    return;
  }
  if (zb_check_joined()) {
    ESP_LOGI(TAG, "Watchdog: looks joined (short 0x%04hx), skipping", esp_zb_get_short_address());
    return;
  }
  ESP_LOGI(TAG, "Attempting to join...");
  (void) param;
  set_joining(true);
  esp_zb_sleep_enable(true);
  esp_zb_set_rx_on_when_idle(false);
  esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_NETWORK_STEERING);
}

static uint32_t s_join_backoff = 15000;
void zb_join_backoff(uint32_t ms) {
  if (ms == 0) {
    s_join_backoff *= 2;
    if (s_join_backoff > 90000) s_join_backoff = 90000;
  } else {
    s_join_backoff = ms;
  }
  ESP_LOGI(TAG, "Joining again in %dms...", s_join_backoff);
  esp_zb_scheduler_alarm(zb_join, 0, s_join_backoff);
}

bool rtos0_active_snapshot(void) {
  char *buf = NULL;
  size_t len = 0;
  FILE *f = open_memstream(&buf, &len);

  esp_pm_dump_locks(f);
  fclose(f);

  bool active = strstr(buf, "rtos0") && strstr(buf, "1");
  free(buf);
  return active;
}

void esp_zb_app_signal_handler(esp_zb_app_signal_t *signal_s) {
  esp_zb_app_signal_type_t sig =
      (signal_s && signal_s->p_app_signal)
        ? *(esp_zb_app_signal_type_t *) signal_s->p_app_signal
        : ESP_ZB_ZDO_SIGNAL_SKIP_STARTUP;

  esp_err_t status = signal_s ? signal_s->esp_err_status : ESP_FAIL;

  switch (sig) {
    case ESP_ZB_ZDO_SIGNAL_SKIP_STARTUP:
      ESP_LOGI(TAG, "Zigbee stack initialized for MAC %s", get_mac_addr());
      esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_INITIALIZATION);
      break;

    case ESP_ZB_BDB_SIGNAL_DEVICE_FIRST_START:
    case ESP_ZB_BDB_SIGNAL_DEVICE_REBOOT: {
      bool factory_new = esp_zb_bdb_is_factory_new();

      ESP_LOGI(TAG, "%s -> factory_new=%d",
               sig == ESP_ZB_BDB_SIGNAL_DEVICE_FIRST_START ? "First start" : "Reboot",
               factory_new);

      if (factory_new) {
        ESP_LOGI(TAG, "Factory new -> steering");
        zb_join_backoff(1500);
      } else if (!zb_check_joined()) {
        ESP_LOGI(TAG, "Not factory new -> wait for rejoin; start watchdog");
        zb_join_backoff(30000);
      }
      break;
    }

    case ESP_ZB_BDB_SIGNAL_STEERING:
      set_joining(false);
      if (status == ESP_OK) {
        on_zb_joined();
      } else {
        ESP_LOGW(TAG, "Steering failed: status=%s short=0x%04x",
        esp_err_to_name(status), esp_zb_get_short_address());
        zb_join_backoff(0);
      }
      break;

    case ESP_ZB_COMMON_SIGNAL_CAN_SLEEP:
        if (is_joined() && is_ready() && !is_reporting()) {
          // ESP_LOGI(TAG, "Zigbee sleeping...?");
          // set_led(rtos0_active_snapshot());
          esp_zb_sleep_now();
        }
      break;

    case ESP_ZB_NLME_STATUS_INDICATION: {
      if (status == ESP_OK) {
        ESP_LOGI(TAG, "Zigbee status indication OK (joined?)");
      } else {
        ESP_LOGW(TAG, "Zigbee negative join status! %s", esp_err_to_name(status));
      }
    } break;

    case ESP_ZB_ZDO_SIGNAL_LEAVE:
      ESP_LOGW(TAG, "Left network; will rejoin shortly.");
      on_left();
      zb_join_backoff(5000);
      break;

    case ESP_ZB_ZDO_SIGNAL_PRODUCTION_CONFIG_READY:
      ESP_LOGI(TAG, "Zigbee production ready? %s", esp_err_to_name(status));
      break;

    case ESP_ZB_ZDO_DEVICE_UNAVAILABLE:
      const esp_zb_zdo_device_unavailable_params_t *p =
          (const esp_zb_zdo_device_unavailable_params_t *)signal_s->p_app_signal;

      if (p) {
        ESP_LOGW(TAG,
                 "ZDO DEVICE_UNAVAILABLE status=%s short=0x%04hx ieee=%02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x",
                 esp_err_to_name(status),
                 p->short_addr,
                 p->long_addr[7], p->long_addr[6], p->long_addr[5], p->long_addr[4],
                 p->long_addr[3], p->long_addr[2], p->long_addr[1], p->long_addr[0]);
      } else {
        ESP_LOGW(TAG, "ZDO DEVICE_UNAVAILABLE status=%s (no params for %d)", esp_err_to_name(status), signal_s->p_app_signal);
      }
      break;

    default:
      ESP_LOGI(TAG, "Zigbee sig: %d status: %s", (int)sig, esp_err_to_name(status));
      break;
  }
}

/* -----------------------------
 * Zigbee task
 * ----------------------------- */

static void zigbee_task(void *pv) {
  (void) pv;

  /* Platform config (ESP32-C6 native radio) */
  esp_zb_platform_config_t platform_config = {
    .radio_config = {
      .radio_mode = ZB_RADIO_MODE_NATIVE,
    },
    .host_config = {
      .host_connection_mode = ZB_HOST_CONNECTION_MODE_NONE,
    },
  };
  ESP_ERROR_CHECK(esp_zb_platform_config(&platform_config));

  esp_zb_cfg_t zb_cfg = {
    .esp_zb_role = ESP_ZB_DEVICE_TYPE_ED,
    .install_code_policy = false,
    .nwk_cfg.zed_cfg = {
      .ed_timeout = ESP_ZB_ED_AGING_TIMEOUT_1024MIN,
      .keep_alive = DEF_REPORTING_MIN_SEC * 1000,
    },
  };

  esp_zb_init(&zb_cfg);
  esp_zb_set_primary_network_channel_set(0x07FFF800); // set all channels
  // esp_zb_zcl_analog_input_init_server();
  esp_zb_zcl_rel_humidity_measurement_init_server();

  esp_zb_ep_list_t *ep_list = esp_zb_ep_list_create();
  {
    esp_zb_endpoint_config_t ep_cfg = {
      .endpoint = EP_WATER,
      .app_profile_id = ESP_ZB_AF_HA_PROFILE_ID,
      .app_device_id = ESP_ZB_HA_SIMPLE_SENSOR_DEVICE_ID,
      .app_device_version = 0,
    };
    esp_zb_cluster_list_t *clusters = create_water_level_clusters(EP_WATER);
    esp_zb_ep_list_add_ep(ep_list, clusters, ep_cfg);
  }

  uint8_t num_zones = get_num_zones();
  for (uint8_t i = 0; i < num_zones; i++) {
    uint8_t ep = get_soil_level_endpoint(i);
    esp_zb_endpoint_config_t sl_cfg = {
      .endpoint = ep,
      .app_profile_id = ESP_ZB_AF_HA_PROFILE_ID,
      .app_device_id = ESP_ZB_HA_SIMPLE_SENSOR_DEVICE_ID,
      .app_device_version = 0,
    };
    esp_zb_cluster_list_t *clusters = create_water_level_clusters(ep);
    esp_zb_ep_list_add_ep(ep_list, clusters, sl_cfg);
  }

  esp_zb_device_register(ep_list);
  esp_zb_nvram_erase_at_start(get_cfg_flag(CFG_FLAG_RESET));

  if (get_cfg_flag(CFG_FLAG_RESET)) {
    ESP_LOGI(TAG, "Resetting due to user flag...");
    set_cfg_flag(CFG_FLAG_RESET, false);
    cfg_save();
  }
  esp_zb_start(false);
  esp_zb_stack_main_loop();
  vTaskDelete(NULL);
}

/* -----------------------------
 * Start & Report Public Interface
 * ----------------------------- */

void zb_report_sensors() {
  uint8_t num_zones = get_num_zones();
  const struct WaterLevel* wl = get_water_level();
  s_water_level_pct_x100 = (uint16_t) wl->percent * 100;
  for (uint8_t i = 0; i < num_zones; i++) {
    const struct SoilLevel* sl = get_soil_level(i);
    s_soil_level_pct_x100[i] = (uint16_t) sl->percent * 100;
  }

  esp_zb_lock_acquire(portMAX_DELAY);
  esp_zb_zcl_set_attribute_val(
    EP_WATER,
    ESP_ZB_ZCL_CLUSTER_ID_REL_HUMIDITY_MEASUREMENT,
    ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
    ESP_ZB_ZCL_ATTR_REL_HUMIDITY_MEASUREMENT_VALUE_ID,
    &s_water_level_pct_x100,
    false
  );
  for (uint8_t i = 0; i < num_zones; i++) {
    esp_zb_zcl_set_attribute_val(
      get_soil_level_endpoint(i),
      ESP_ZB_ZCL_CLUSTER_ID_REL_HUMIDITY_MEASUREMENT,
      ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
      ESP_ZB_ZCL_ATTR_REL_HUMIDITY_MEASUREMENT_VALUE_ID,
      &s_soil_level_pct_x100[i],
      false
    );
  }
  esp_zb_lock_release();
}

static void zb_send_battery_reports_now(void){
  esp_zb_zcl_report_attr_cmd_t cmd = {0};

  cmd.zcl_basic_cmd.dst_addr_u.addr_short = COORDINATOR_SHORT_ADDR;
  cmd.zcl_basic_cmd.src_endpoint = EP_WATER;
  cmd.zcl_basic_cmd.dst_endpoint = COORDINATOR_ENDPOINT;
  cmd.address_mode = ESP_ZB_APS_ADDR_MODE_16_ENDP_PRESENT;

  // Server -> Client report (matches Espressif doc example usage)
  cmd.direction = ESP_ZB_ZCL_CMD_DIRECTION_TO_CLI;
  cmd.clusterID  = ESP_ZB_ZCL_CLUSTER_ID_POWER_CONFIG;

  // Report BatteryVoltage
  cmd.attributeID = ESP_ZB_ZCL_ATTR_POWER_CONFIG_BATTERY_VOLTAGE_ID;
  esp_zb_zcl_report_attr_cmd_req(&cmd);

  // Report BatteryPercentageRemaining
  cmd.attributeID = ESP_ZB_ZCL_ATTR_POWER_CONFIG_BATTERY_PERCENTAGE_REMAINING_ID;
  esp_zb_zcl_report_attr_cmd_req(&cmd);
}

void zb_report_battery() {
  zb_get_battery();
  esp_zb_lock_acquire(portMAX_DELAY);
  esp_zb_zcl_set_attribute_val(
      EP_WATER,
      ESP_ZB_ZCL_CLUSTER_ID_POWER_CONFIG,
      ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
      ESP_ZB_ZCL_ATTR_POWER_CONFIG_BATTERY_VOLTAGE_ID,
      &s_battery_voltage,
      false
      );
  esp_zb_zcl_set_attribute_val(
      EP_WATER,
      ESP_ZB_ZCL_CLUSTER_ID_POWER_CONFIG,
      ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
      ESP_ZB_ZCL_ATTR_POWER_CONFIG_BATTERY_PERCENTAGE_REMAINING_ID,
      &s_battery_percent,
      false
  );
  zb_send_battery_reports_now();
  esp_zb_lock_release();
}

void zb_start(void) {
  // vTaskDelay(pdMS_TO_TICKS(1000));
  xTaskCreate(zigbee_task, "zigbee_task", 0x2000, NULL, 5, NULL);
}
