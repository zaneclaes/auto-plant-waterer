#include <string.h>
#include <inttypes.h>

#include "zb.h"
#include "cfg.h"
#include "bat.h"
#include "tof.h"
#include "pumps.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"

#include "esp_log.h"
#include "esp_err.h"

#include "driver/gpio.h"
#include "driver/i2c.h"

/* Zigbee */
#include "esp_check.h"
#include "esp_zigbee_core.h"
#include "zcl/esp_zigbee_zcl_common.h"
#include "zcl/esp_zigbee_zcl_command.h"
// #include "zcl/esp_zigbee_zcl_analog_input.h"
#include "esp_pm.h"
#include "esp_timer.h"
#include "nvs_flash.h"
#include "zcl/esp_zigbee_zcl_humidity_meas.h"
#include "zcl/esp_zigbee_zcl_power_config.h"

static const char *TAG = "zb";


/* -----------------------------
 * USER CONFIG
 * ----------------------------- */
#define REPORTING_INTERVAL_MS 60000

#define COORDINATOR_SHORT_ADDR 0x0000
#define COORDINATOR_ENDPOINT   1   // Z2M coordinator endpoint is typically 1

#define ZB_ALL_CHANNELS_MASK 0x07FFF800UL

#define ZB_JOINED_BIT  BIT0

/* Zigbee endpoints */
#define EP_WATER      0x01
#define EP_PUMP1      0x02
#define EP_PUMP2      0x03
#define EP_PUMP3      0x04

/* Custom cluster for water level (must be >= ESP_ZB_CUSTOM_CLUSTER_ID_MIN_VAL) */
#define WATER_CLUSTER_ID              0xFF01
#define WATER_ATTR_WATER_LEVEL_MM_ID  0x0000

/* -----------------------------
 * INTERNALS
 * ----------------------------- */

static EventGroupHandle_t s_zb_events;

static uint16_t s_water_level_pct_x100 = 0; // 0..10000  (0.01% units)
static uint16_t s_rh_min_x100 = 0; // 0.00%
static uint16_t s_rh_max_x100 = 10000; // 100.00%

static uint8_t s_battery_voltage = 0;
static uint8_t s_battery_percent = 0;


// #include "zdo/esp_zigbee_zdo_command.h"   // esp_zb_zdo_active_scan_request
//
// static void active_scan_cb(esp_zb_zdp_status_t zdo_status,
//                            uint8_t count,
//                            esp_zb_network_descriptor_t *nwk_descriptor)
// {
//   ESP_LOGW(TAG, "ACTIVE_SCAN done: zdo_status=%d count=%u", zdo_status, count);
//
//   if (zdo_status != ESP_ZB_ZDP_STATUS_SUCCESS || !nwk_descriptor || count == 0) {
//     ESP_LOGW(TAG, "ACTIVE_SCAN: no networks found");
//     return;
//   }
//
//   for (uint8_t i = 0; i < count; i++) {
//     const esp_zb_network_descriptor_t *d = &nwk_descriptor[i];
//
//     ESP_LOGW(TAG,
//              "  net[%u]: pan=0x%04x ch=%u permit_join=%u router_cap=%u enddev_cap=%u ext_pan=%02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x",
//              i,
//              d->short_pan_id,
//              d->logic_channel,
//              (unsigned)d->permit_joining,
//              (unsigned)d->router_capacity,
//              (unsigned)d->end_device_capacity,
//              d->extended_pan_id[7], d->extended_pan_id[6], d->extended_pan_id[5], d->extended_pan_id[4],
//              d->extended_pan_id[3], d->extended_pan_id[2], d->extended_pan_id[1], d->extended_pan_id[0]);
//   }
// }
//
// static void do_active_scan(void)
// {
//   ESP_LOGW(TAG, "Starting ACTIVE_SCAN...");
//   esp_zb_zdo_active_scan_request(ZB_ALL_CHANNELS_MASK, 4, active_scan_cb);
// }

/* -----------------------------
 * LOCK; prevent power management while joining...
 * ----------------------------- */

static esp_pm_lock_handle_t s_pm_no_ls_lock;
static esp_pm_lock_handle_t s_pm_cpu_max_lock;
static esp_timer_handle_t s_release_timer;

static void pm_lock_hold_for_join(void) {
  ESP_LOGI(TAG, "Pausing power management...");
  if (!s_pm_no_ls_lock) {
    ESP_ERROR_CHECK(esp_pm_lock_create(ESP_PM_NO_LIGHT_SLEEP, 0, "zb_join", &s_pm_no_ls_lock));
  }
  if (!s_pm_cpu_max_lock) {
    ESP_ERROR_CHECK(esp_pm_lock_create(ESP_PM_CPU_FREQ_MAX, 0, "zb_join_cpu", &s_pm_cpu_max_lock));
  }
  ESP_ERROR_CHECK(esp_pm_lock_acquire(s_pm_no_ls_lock));
  ESP_ERROR_CHECK(esp_pm_lock_acquire(s_pm_cpu_max_lock));
}

static void pm_lock_release_after_join(void *arg) {
  if (s_pm_no_ls_lock) {
    ESP_LOGI(TAG, "Resuming power management...");
    esp_pm_lock_release(s_pm_no_ls_lock);
  }
  if (s_pm_cpu_max_lock) esp_pm_lock_release(s_pm_cpu_max_lock);
}

static void start_release_timer_ms(uint32_t ms) {
  if (!s_release_timer) {
    const esp_timer_create_args_t targs = {
      .callback = &pm_lock_release_after_join,
      .name = "zb_release_ls",
    };
    ESP_ERROR_CHECK(esp_timer_create(&targs, &s_release_timer));
  }

  ESP_LOGI(TAG, "Starting power management timer...");
  esp_timer_stop(s_release_timer);
  ESP_ERROR_CHECK(esp_timer_start_once(s_release_timer, (uint64_t)ms * 1000ULL));
}

/* -----------------------------
 * GPIO / PUMPS
 * ----------------------------- */

static void zb_update_battery(void) {
  const struct BatteryLevel *bat = battery_update();
  s_battery_percent = bat->percent * 2;
  s_battery_voltage = (uint8_t) (bat->voltage * 10.0f);
}

static uint8_t zb_get_pump_idx(uint8_t endpoint) {
  if (endpoint == EP_PUMP1) return 0;
  if (endpoint == EP_PUMP2) return 1;
  if (endpoint == EP_PUMP3) return 2;

  ESP_LOGW(TAG, "Pump endpoint %u is unknown; assuming idx 0", endpoint);
  return 0;
}

static void zb_update_tof(void) {
  const struct WaterLevel *water = tof_update();
  s_water_level_pct_x100 = (uint16_t) (water->percent * 100);
}

/* -----------------------------
 * I2C / ToF (stub)
 * ----------------------------- */

static uint16_t humidity_reportable_change = 100; // 1.00% (units are 0.01%)
static void bind_humidity_cb(esp_zb_zdp_status_t zdo_status, void *user_ctx) {
  esp_zb_zdo_bind_req_param_t *bind_req = (esp_zb_zdo_bind_req_param_t *) user_ctx;

  if (zdo_status == ESP_ZB_ZDP_STATUS_SUCCESS) {
    ESP_LOGI(TAG, "Bind OK: humidity reports from ep %d to coordinator ep %d",
             bind_req->src_endp, bind_req->dst_endp);

    esp_zb_zcl_config_report_cmd_t report_cmd = {0};

    report_cmd.zcl_basic_cmd.dst_addr_u.addr_short = COORDINATOR_SHORT_ADDR;
    //esp_zb_get_short_address(); // local config
    report_cmd.zcl_basic_cmd.src_endpoint = EP_WATER;
    report_cmd.zcl_basic_cmd.dst_endpoint = EP_WATER;
    report_cmd.address_mode = ESP_ZB_APS_ADDR_MODE_16_ENDP_PRESENT;

    report_cmd.clusterID = ESP_ZB_ZCL_CLUSTER_ID_REL_HUMIDITY_MEASUREMENT;

    esp_zb_zcl_config_report_record_t rec = {
      .direction = ESP_ZB_ZCL_REPORT_DIRECTION_SEND,
      .attributeID = ESP_ZB_ZCL_ATTR_REL_HUMIDITY_MEASUREMENT_VALUE_ID,
      .attrType = ESP_ZB_ZCL_ATTR_TYPE_U16,

      // Pick sane defaults for a product:
      .min_interval = 5, // no more often than every 5s
      .max_interval = 60, // at least once a minute even if stable
      .reportable_change = &humidity_reportable_change, // 1.00% change
    };

    report_cmd.record_number = 1;
    report_cmd.record_field = &rec;

    esp_zb_lock_acquire(portMAX_DELAY);
    esp_zb_zcl_config_report_cmd_req(&report_cmd);
    esp_zb_lock_release();

    ESP_LOGI(TAG, "Local reporting configured (min=5s max=60s change=1%%)");
  } else {
    ESP_LOGW(TAG, "Bind failed: zdo_status=%d", zdo_status);
  }

  free(bind_req);
}

static void humidity_bind_to_coordinator(void) {
  esp_zb_zdo_bind_req_param_t *bind_req =
      (esp_zb_zdo_bind_req_param_t *) calloc(1, sizeof(esp_zb_zdo_bind_req_param_t));

  bind_req->req_dst_addr = COORDINATOR_SHORT_ADDR; // esp_zb_get_short_address(); // local device
  bind_req->src_endp = EP_WATER; // <-- your sensor endpoint
  bind_req->dst_endp = COORDINATOR_ENDPOINT;
  bind_req->cluster_id = ESP_ZB_ZCL_CLUSTER_ID_REL_HUMIDITY_MEASUREMENT;

  bind_req->dst_addr_mode = ESP_ZB_ZDO_BIND_DST_ADDR_MODE_64_BIT_EXTENDED;

  // Destination IEEE = coordinator IEEE (looked up by short)
  esp_zb_ieee_address_by_short(COORDINATOR_SHORT_ADDR, bind_req->dst_address_u.addr_long);

  // Source IEEE = our IEEE
  esp_zb_get_long_address(bind_req->src_address);

  esp_zb_zdo_device_bind_req(bind_req, bind_humidity_cb, bind_req);
}

static uint8_t battery_pct_reportable_change = 2; // 1% (0.5% units)
static uint8_t battery_v_reportable_change = 1; // 0.1V (100mV units)

static void bind_battery_cb(esp_zb_zdp_status_t zdo_status, void *user_ctx) {
  esp_zb_zdo_bind_req_param_t *bind_req = (esp_zb_zdo_bind_req_param_t *) user_ctx;

  if (zdo_status == ESP_ZB_ZDP_STATUS_SUCCESS) {
    ESP_LOGI(TAG, "Bind OK: battery reports from ep %d to coordinator ep %d",
             bind_req->src_endp, bind_req->dst_endp);

    esp_zb_zcl_config_report_cmd_t cmd = {0};

    cmd.address_mode = ESP_ZB_APS_ADDR_MODE_16_ENDP_PRESENT;
    cmd.zcl_basic_cmd.dst_addr_u.addr_short = COORDINATOR_SHORT_ADDR; // esp_zb_get_short_address();
    cmd.zcl_basic_cmd.src_endpoint = EP_WATER;
    cmd.zcl_basic_cmd.dst_endpoint = EP_WATER;
    cmd.clusterID = ESP_ZB_ZCL_CLUSTER_ID_POWER_CONFIG;

    esp_zb_zcl_config_report_record_t recs[2] = {
      {
        .direction = ESP_ZB_ZCL_REPORT_DIRECTION_SEND,
        .attributeID = ESP_ZB_ZCL_ATTR_POWER_CONFIG_BATTERY_PERCENTAGE_REMAINING_ID,
        .attrType = ESP_ZB_ZCL_ATTR_TYPE_U8,
        .min_interval = 300,
        .max_interval = 3600,
        .reportable_change = &battery_pct_reportable_change,
      },
      {
        .direction = ESP_ZB_ZCL_REPORT_DIRECTION_SEND,
        .attributeID = ESP_ZB_ZCL_ATTR_POWER_CONFIG_BATTERY_VOLTAGE_ID,
        .attrType = ESP_ZB_ZCL_ATTR_TYPE_U8,
        .min_interval = 300,
        .max_interval = 3600,
        .reportable_change = &battery_v_reportable_change,
      }
    };

    cmd.record_number = 2;
    cmd.record_field = recs;

    esp_zb_lock_acquire(portMAX_DELAY);
    esp_zb_zcl_config_report_cmd_req(&cmd);
    esp_zb_lock_release();
  } else {
    ESP_LOGW(TAG, "Bind failed: zdo_status=%d", zdo_status);
  }

  free(bind_req);
}

static void battery_bind_to_coordinator() {
  esp_zb_zdo_bind_req_param_t *bind_req =
      calloc(1, sizeof(esp_zb_zdo_bind_req_param_t));

  bind_req->req_dst_addr = COORDINATOR_SHORT_ADDR; // esp_zb_get_short_address(); // local device
  bind_req->src_endp = EP_WATER;
  bind_req->dst_endp = 1; // coordinator endpoint
  bind_req->cluster_id = ESP_ZB_ZCL_CLUSTER_ID_POWER_CONFIG;
  bind_req->dst_addr_mode = ESP_ZB_ZDO_BIND_DST_ADDR_MODE_64_BIT_EXTENDED;

  esp_zb_get_long_address(bind_req->src_address);
  esp_zb_ieee_address_by_short(0x0000, bind_req->dst_address_u.addr_long);

  esp_zb_zdo_device_bind_req(bind_req, bind_battery_cb, bind_req);
}


/* -----------------------------
 * Zigbee: device model creation
 * ----------------------------- */

static esp_zb_cluster_list_t *create_onoff_switch_clusters(bool initial_on) {
  esp_zb_cluster_list_t *cluster_list = esp_zb_zcl_cluster_list_create();

  /* Basic cluster */
  esp_zb_basic_cluster_cfg_t basic_cfg = {0};
  esp_zb_attribute_list_t *basic = esp_zb_basic_cluster_create(&basic_cfg);

  /* Add some common Basic attributes (manufacturer/model) */
  esp_zb_basic_cluster_add_attr(basic, ESP_ZB_ZCL_ATTR_BASIC_MANUFACTURER_NAME_ID, (void *) ZB_MANUFACTURER_NAME);
  esp_zb_basic_cluster_add_attr(basic, ESP_ZB_ZCL_ATTR_BASIC_MODEL_IDENTIFIER_ID, (void *) ZB_MODEL_IDENTIFIER);

  esp_zb_cluster_list_add_basic_cluster(cluster_list, basic, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);

  /* Identify cluster */
  esp_zb_identify_cluster_cfg_t identify_cfg = {0};
  esp_zb_attribute_list_t *identify = esp_zb_identify_cluster_create(&identify_cfg);
  esp_zb_cluster_list_add_identify_cluster(cluster_list, identify, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);

  /* On/Off cluster */
  esp_zb_on_off_cluster_cfg_t onoff_cfg = {0};
  onoff_cfg.on_off = initial_on ? 1 : 0;
  esp_zb_attribute_list_t *onoff = esp_zb_on_off_cluster_create(&onoff_cfg);
  esp_zb_cluster_list_add_on_off_cluster(cluster_list, onoff, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);

  return cluster_list;
}

static esp_zb_cluster_list_t *create_water_level_clusters(void) {
  esp_zb_cluster_list_t *cluster_list = esp_zb_zcl_cluster_list_create();

  /* Basic cluster */
  esp_zb_basic_cluster_cfg_t basic_cfg = {0};
  esp_zb_attribute_list_t *basic = esp_zb_basic_cluster_create(&basic_cfg);
  esp_zb_basic_cluster_add_attr(basic, ESP_ZB_ZCL_ATTR_BASIC_MANUFACTURER_NAME_ID, (void *) ZB_MANUFACTURER_NAME);
  esp_zb_basic_cluster_add_attr(basic, ESP_ZB_ZCL_ATTR_BASIC_MODEL_IDENTIFIER_ID, (void *) ZB_MODEL_IDENTIFIER);
  esp_zb_cluster_list_add_basic_cluster(cluster_list, basic, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);

  /* Identify cluster */
  esp_zb_identify_cluster_cfg_t identify_cfg = {0};
  esp_zb_attribute_list_t *identify = esp_zb_identify_cluster_create(&identify_cfg);
  esp_zb_cluster_list_add_identify_cluster(cluster_list, identify, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);

  /* ---- Analog Input cluster (0x000C) ----
     IMPORTANT: Build a *standard* Analog Input cluster attribute list, then add it
     using the SDK's Analog Input cluster adder (NOT "custom cluster").
  */
  esp_zb_attribute_list_t *rh =
      esp_zb_zcl_attr_list_create(ESP_ZB_ZCL_CLUSTER_ID_REL_HUMIDITY_MEASUREMENT);

  esp_zb_custom_cluster_add_custom_attr(
    rh,
    ESP_ZB_ZCL_ATTR_REL_HUMIDITY_MEASUREMENT_VALUE_ID,
    ESP_ZB_ZCL_ATTR_TYPE_U16,
    ESP_ZB_ZCL_ATTR_ACCESS_READ_ONLY | ESP_ZB_ZCL_ATTR_ACCESS_REPORTING,
    &s_water_level_pct_x100
  );

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


  /* Add as a STANDARD humidity cluster */
  esp_zb_cluster_list_add_humidity_meas_cluster(cluster_list, rh, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);

  zb_update_battery();
  esp_zb_attribute_list_t *pwr =
      esp_zb_zcl_attr_list_create(ESP_ZB_ZCL_CLUSTER_ID_POWER_CONFIG);

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

  return cluster_list;
}

static void zigbee_create_endpoints_and_register(void) {
  esp_zb_ep_list_t *ep_list = esp_zb_ep_list_create();

  /* Endpoint config templates */
  esp_zb_endpoint_config_t pump_ep_cfg = {
    .app_profile_id = ESP_ZB_AF_HA_PROFILE_ID,
    .app_device_id = ESP_ZB_HA_ON_OFF_LIGHT_DEVICE_ID, /* Reuse HA On/Off Light for pump switch */
    .app_device_version = 0,
  };

  /* Pump 1 */
  {
    esp_zb_endpoint_config_t ep_cfg = pump_ep_cfg;
    ep_cfg.endpoint = EP_PUMP1;
    esp_zb_cluster_list_t *clusters = create_onoff_switch_clusters(false);
    esp_zb_ep_list_add_ep(ep_list, clusters, ep_cfg);
  }
  /* Pump 2 */
  {
    esp_zb_endpoint_config_t ep_cfg = pump_ep_cfg;
    ep_cfg.endpoint = EP_PUMP2;
    esp_zb_cluster_list_t *clusters = create_onoff_switch_clusters(false);
    esp_zb_ep_list_add_ep(ep_list, clusters, ep_cfg);
  }
  /* Pump 3 */
  {
    esp_zb_endpoint_config_t ep_cfg = pump_ep_cfg;
    ep_cfg.endpoint = EP_PUMP3;
    esp_zb_cluster_list_t *clusters = create_onoff_switch_clusters(false);
    esp_zb_ep_list_add_ep(ep_list, clusters, ep_cfg);
  }

  /* Water level endpoint (custom attribute device) */
  {
    esp_zb_endpoint_config_t ep_cfg = {
      .endpoint = EP_WATER,
      .app_profile_id = ESP_ZB_AF_HA_PROFILE_ID,
      .app_device_id = ESP_ZB_HA_SIMPLE_SENSOR_DEVICE_ID,
      .app_device_version = 0,
    };
    esp_zb_cluster_list_t *clusters = create_water_level_clusters();
    esp_zb_ep_list_add_ep(ep_list, clusters, ep_cfg);
  }

  esp_zb_device_register(ep_list);
}

/* -----------------------------
 * Zigbee: action callback (ZCL set attribute)
 * ----------------------------- */

static esp_err_t zb_action_handler(esp_zb_core_action_callback_id_t callback_id, const void *message) {
  switch (callback_id) {
    case ESP_ZB_CORE_SET_ATTR_VALUE_CB_ID: {
      const esp_zb_zcl_set_attr_value_message_t *m = (const esp_zb_zcl_set_attr_value_message_t *) message;
      if (!m) return ESP_FAIL;

      /* Common info includes dst endpoint, cluster id, etc. */
      uint8_t endpoint = m->info.dst_endpoint;
      uint16_t cluster_id = m->info.cluster;

      /* We only care about On/Off cluster writes */
      if (cluster_id == ESP_ZB_ZCL_CLUSTER_ID_ON_OFF) {
        if (m->attribute.id == ESP_ZB_ZCL_ATTR_ON_OFF_ON_OFF_ID && m->attribute.data.type ==
            ESP_ZB_ZCL_ATTR_TYPE_BOOL) {
          bool on = (*(bool *) m->attribute.data.value) ? true : false;
          pump_set(zb_get_pump_idx(endpoint), on);

          /* Keep ZCL attribute store consistent with physical state */
          esp_zb_lock_acquire(portMAX_DELAY);
          esp_zb_zcl_set_attribute_val(
            endpoint,
            ESP_ZB_ZCL_CLUSTER_ID_ON_OFF,
            ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
            ESP_ZB_ZCL_ATTR_ON_OFF_ON_OFF_ID,
            &on,
            false
          );
          esp_zb_lock_release();
        }
      }
      return ESP_OK;
    }

    default:
      /* You’ll see other callbacks here (reports, scenes, etc.) if enabled */
      return ESP_OK;
  }
}

/* -----------------------------
 * Zigbee: signal handler (commissioning / join)
 * ----------------------------- */
static bool s_bdb_inited = false;
static bool s_steering_in_progress = false;

static void zb_start_steering(void) {
  if (s_steering_in_progress) {
    ESP_LOGW(TAG, "Steering already in progress; skip");
    return;
  }
  s_steering_in_progress = true;
  ESP_LOGI(TAG, "Starting BDB network steering...");
  esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_NETWORK_STEERING);
}

static void zb_start_bdb_init(void) {
  if (s_bdb_inited) {
    ESP_LOGI(TAG, "BDB already initialized; skip init");
    return;
  }
  ESP_LOGI(TAG, "Starting BDB initialization...");
  esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_INITIALIZATION);
}

static void zb_retry_commissioning(uint8_t param) {
  (void) param;
  s_steering_in_progress = false; // allow retry
  zb_start_steering();
}

static void verify_join_later(uint8_t arg) {
  (void) arg;

  bool joined = esp_zb_bdb_dev_joined();
  uint16_t pan = esp_zb_get_pan_id();
  uint16_t short_addr = esp_zb_get_short_address();
  uint8_t ch = esp_zb_get_current_channel();

  ESP_LOGW(TAG, "JOIN_VERIFY: joined=%d pan=0x%04x short=0x%04x ch=%u",
           joined, pan, short_addr, ch);

  if (!joined || pan == 0xFFFF || short_addr == 0xFFFE) {
    ESP_LOGW(TAG, "JOIN_VERIFY: not actually joined; retry steering");
    s_steering_in_progress = false;
    esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_NETWORK_STEERING);
    return;
  }

  ESP_LOGI(TAG, "JOIN_VERIFY: confirmed joined; enabling sleep + binds");
  xEventGroupSetBits(s_zb_events, ZB_JOINED_BIT);

  esp_zb_sleep_enable(true);
  esp_zb_set_rx_on_when_idle(false);

  humidity_bind_to_coordinator();
  battery_bind_to_coordinator();
  start_release_timer_ms(120000);
}

static void reboot_later(uint8_t arg){
  (void)arg;
  ESP_LOGE(TAG, "Rebooting due to Zigbee startup failure...");
  esp_restart();
}

void esp_zb_app_signal_handler(esp_zb_app_signal_t *signal_s) {
  /* esp-zigbee-lib 1.6.x pattern */
  esp_zb_app_signal_msg_t *m =
      (signal_s) ? (esp_zb_app_signal_msg_t *) signal_s->p_app_signal : NULL;

  esp_zb_app_signal_type_t sig =
      (m) ? m->signal : ESP_ZB_ZDO_SIGNAL_SKIP_STARTUP;

  esp_err_t status = signal_s ? signal_s->esp_err_status : ESP_FAIL;

  switch (sig) {
    case ESP_ZB_ZDO_SIGNAL_SKIP_STARTUP:
      ESP_LOGI(TAG, "SKIP_STARTUP (BDB init)");
      zb_start_bdb_init();
      break;

    case ESP_ZB_BDB_SIGNAL_DEVICE_FIRST_START:
    case ESP_ZB_BDB_SIGNAL_DEVICE_REBOOT:
      if (status != ESP_OK) {
        ESP_LOGW(TAG, "Device start/reboot failed (%s) -> retry steering soon",
                 esp_err_to_name(status));
        s_steering_in_progress = false;
        esp_zb_scheduler_alarm(zb_retry_commissioning, 0, 1500);
        break;
      }

      if (esp_zb_bdb_is_factory_new()) {
        ESP_LOGI(TAG, "Factory new -> start steering");
        zb_start_steering();
      } else {
        ESP_LOGI(TAG, "Not factory new -> waiting for rejoin");
      }
      break;

    case ESP_ZB_BDB_SIGNAL_STEERING:
        if (status == ESP_OK) {
            ESP_LOGI(TAG, "Joined network OK (PAN: 0x%04hx, short: 0x%04hx)",
                     esp_zb_get_pan_id(), esp_zb_get_short_address());
            xEventGroupSetBits(s_zb_events, ZB_JOINED_BIT);

            esp_zb_sleep_enable(true);
            esp_zb_set_rx_on_when_idle(false);

            humidity_bind_to_coordinator();
            battery_bind_to_coordinator();
            // start_release_timer_ms(120000);
        } else {
            ESP_LOGW(TAG, "Steering failed (%s). Retrying...", esp_err_to_name(status));
            /* schedule retry (see wrapper in section 3) */
            esp_zb_scheduler_alarm(zb_retry_commissioning, 0, 3000);
        }
        break;

    case ESP_ZB_NLME_STATUS_INDICATION: {
      esp_zb_zdo_signal_nwk_status_indication_params_t *p =
          (esp_zb_zdo_signal_nwk_status_indication_params_t *) esp_zb_app_signal_get_params(signal_s->p_app_signal);

      if (p) {
        ESP_LOGW(TAG, "NLME_STATUS_INDICATION: nwk_status=0x%02x network_addr=0x%04x unknown_cmd=0x%02x",
                 p->status, p->network_addr, p->unknown_command_id);
      } else {
        ESP_LOGW(TAG, "NLME_STATUS_INDICATION: (no params)");
      }
      break;
    }

    case ESP_ZB_ZDO_SIGNAL_LEAVE:
      ESP_LOGW(TAG, "Left network");
      xEventGroupClearBits(s_zb_events, ZB_JOINED_BIT);
      esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_NETWORK_STEERING);
      break;

    case ESP_ZB_COMMON_SIGNAL_CAN_SLEEP:
      EventBits_t bits = xEventGroupGetBits(s_zb_events);
      if (bits & ZB_JOINED_BIT) {
        ESP_LOGI(TAG, "Zigbee sleeping...?");
        esp_zb_sleep_now();
      }
      break;

    default:
      ESP_LOGI(TAG, "Zigbee sig: %d status: %s msg: %s",
               (int)sig, esp_err_to_name(status), (m && m->msg) ? m->msg : "(none)");
      ESP_LOGW(TAG, "p_app_signal=%p", signal_s ? signal_s->p_app_signal : NULL);
      if (signal_s && signal_s->p_app_signal) {
        uint8_t *b = (uint8_t *)signal_s->p_app_signal;
        ESP_LOGW(TAG, "p_app_signal bytes: %02x %02x %02x %02x %02x %02x %02x %02x",
                 b[0], b[1], b[2], b[3], b[4], b[5], b[6], b[7]);
      }
      break;
  }
}

/* -----------------------------
 * Zigbee task
 * ----------------------------- */
static void zigbee_task(void *pv) {
  (void) pv;

  esp_zb_platform_config_t platform_config = {
    .radio_config = {  .radio_mode = ZB_RADIO_MODE_NATIVE  },
    .host_config = {  .host_connection_mode = ZB_HOST_CONNECTION_MODE_NONE },
  };
  ESP_ERROR_CHECK(esp_zb_platform_config(&platform_config));

  esp_zb_cfg_t zb_cfg = {
    .esp_zb_role = ESP_ZB_DEVICE_TYPE_ED,
    .install_code_policy = false,
    .nwk_cfg.zed_cfg = {
      .ed_timeout = ESP_ZB_ED_AGING_TIMEOUT_1024MIN,
      .keep_alive = 3000, // polling/keep-alive baseline
    },
  };

  // esp_zb_nvram_erase_at_start(true); // erase NVRAM
  // pm_lock_hold_for_join();
  // esp_zb_sleep_enable(false);
  // esp_zb_set_rx_on_when_idle(true);

  esp_log_level_set("phy", ESP_LOG_DEBUG);
  esp_log_level_set("ieee802154", ESP_LOG_DEBUG);
  esp_log_level_set("zb", ESP_LOG_DEBUG);
  esp_log_level_set("*", ESP_LOG_INFO);
  esp_zb_init(&zb_cfg);

  // esp_zb_bdb_set_scan_duration(6);
  // ESP_ERROR_CHECK(esp_zb_set_primary_network_channel_set(ZB_ALL_CHANNELS_MASK));
  // uint32_t chmask = esp_zb_get_primary_network_channel_set();
  // ESP_LOGI(TAG, "BDB primary channel mask = 0x%08"PRIx32, chmask);

  // esp_zb_zcl_rel_humidity_measurement_init_server();
  // zigbee_create_endpoints_and_register();
  // esp_zb_core_action_handler_register(zb_action_handler);

  ESP_ERROR_CHECK(esp_zb_start(false));
  esp_zb_stack_main_loop();
  vTaskDelete(NULL);
}

/* -----------------------------
 * ToF task: update Zigbee attribute
 * ----------------------------- */

static void tof_task(void *pv) {
  /* Wait until joined before reporting/updating attributes */
  xEventGroupWaitBits(s_zb_events, ZB_JOINED_BIT, pdFALSE, pdTRUE, portMAX_DELAY);

  while (true) {
    zb_update_tof();

    esp_zb_lock_acquire(portMAX_DELAY);
    esp_zb_zcl_set_attribute_val(
      EP_WATER,
      ESP_ZB_ZCL_CLUSTER_ID_REL_HUMIDITY_MEASUREMENT,
      ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
      ESP_ZB_ZCL_ATTR_REL_HUMIDITY_MEASUREMENT_VALUE_ID,
      &s_water_level_pct_x100,
      false
    );
    esp_zb_lock_release();

    vTaskDelay(pdMS_TO_TICKS(REPORTING_INTERVAL_MS));
  }
}

static void battery_task(void *pv) {
  /* Wait until joined before reporting/updating attributes */
  xEventGroupWaitBits(s_zb_events, ZB_JOINED_BIT, pdFALSE, pdTRUE, portMAX_DELAY);

  while (true) {
    zb_update_battery();
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
    esp_zb_lock_release();

    vTaskDelay(pdMS_TO_TICKS(60000 * 30)); // 30 min battery reporting
  }
}

/* -----------------------------
 * app_main
 * ----------------------------- */

void zb_start(void) {
  ESP_ERROR_CHECK(nvs_flash_init());
  s_zb_events = xEventGroupCreate();
  xTaskCreate(zigbee_task, "zigbee_task", 8192, NULL, 5, NULL);
  // xTaskCreate(tof_task, "tof_task", 4096, NULL, 4, NULL);
  // xTaskCreate(battery_task, "battery_task", 4096, NULL, 4, NULL);
}
