#include <string.h>
#include <inttypes.h>
#include "zb.h"
#include "cfg.h"
#include "tof.h"
#include "water_pumps.h"

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
#include "zcl/esp_zigbee_zcl_humidity_meas.h"

/* TAG */
static const char *TAG = "zb";

/* -----------------------------
 * USER CONFIG
 * ----------------------------- */
#define COORDINATOR_SHORT_ADDR 0x0000
#define COORDINATOR_ENDPOINT   1   // Z2M coordinator endpoint is typically 1

#define MIN_REPORTING_SEC   60
#define MAX_REPORTING_SEC   600

/* Pump GPIOs (set to your pins) */

/* I2C for ToF sensor (set to your pins) */
/* If you use VL53L0X/VL53L1X etc, set its I2C address here */

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
#define ZB_JOINED_BIT  BIT0

static uint16_t s_water_level_pct_x100 = 0; // 0..10000  (0.01% units)
static uint16_t s_rh_min_x100 = 0; // 0.00%
static uint16_t s_rh_max_x100 = 10000; // 100.00%

static const gpio_num_t s_pump_gpios[3] = {PUMP1_GPIO, PUMP2_GPIO, PUMP3_GPIO};

/* Forward decls */
static void zigbee_task(void *pv);

static void tof_task(void *pv);

static void pumps_gpio_init(void);

static void pump_set_by_endpoint(uint8_t endpoint, bool on);

/* Zigbee handlers */
static esp_err_t zb_action_handler(esp_zb_core_action_callback_id_t callback_id, const void *message);
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

static void pumps_gpio_init(void) {
  gpio_config_t cfg = {
    .mode = GPIO_MODE_OUTPUT,
    .pin_bit_mask =
    (1ULL << PUMP1_GPIO) |
    (1ULL << PUMP2_GPIO) |
    (1ULL << PUMP3_GPIO),
    .pull_down_en = GPIO_PULLDOWN_DISABLE,
    .pull_up_en = GPIO_PULLUP_DISABLE,
    .intr_type = GPIO_INTR_DISABLE,
  };
  ESP_ERROR_CHECK(gpio_config(&cfg));

  /* Default OFF */
  gpio_set_level(PUMP1_GPIO, 0);
  gpio_set_level(PUMP2_GPIO, 0);
  gpio_set_level(PUMP3_GPIO, 0);
}

static void pump_set_by_endpoint(uint8_t endpoint, bool on) {
  int idx = -1;
  if (endpoint == EP_PUMP1) idx = 0;
  else if (endpoint == EP_PUMP2) idx = 1;
  else if (endpoint == EP_PUMP3) idx = 2;

  if (idx < 0) return;

  gpio_set_level(s_pump_gpios[idx], on ? 1 : 0);
  ESP_LOGI(TAG, "Pump endpoint %u -> %s (GPIO %d)", endpoint, on ? "ON" : "OFF", (int)s_pump_gpios[idx]);
}

/* -----------------------------
 * I2C / ToF (stub)
 * ----------------------------- */

/* Reporting thresholds */
static uint16_t s_reportable_change = 100; // 1.00% (units are 0.01%)
static void humidity_local_config_reporting(void);

static void bind_cb(esp_zb_zdp_status_t zdo_status, void *user_ctx) {
  esp_zb_zdo_bind_req_param_t *bind_req = (esp_zb_zdo_bind_req_param_t *) user_ctx;

  if (zdo_status == ESP_ZB_ZDP_STATUS_SUCCESS) {
    ESP_LOGI(TAG, "Bind OK: humidity reports from ep %d to coordinator ep %d",
             bind_req->src_endp, bind_req->dst_endp);

    // Once bound, configure local reporting rules for MeasuredValue
    humidity_local_config_reporting();
  } else {
    ESP_LOGW(TAG, "Bind failed: zdo_status=%d", zdo_status);
  }

  free(bind_req);
}

static void humidity_bind_to_coordinator(void) {
  esp_zb_zdo_bind_req_param_t *bind_req =
      (esp_zb_zdo_bind_req_param_t *) calloc(1, sizeof(esp_zb_zdo_bind_req_param_t));

  bind_req->req_dst_addr = esp_zb_get_short_address(); // local device
  bind_req->src_endp = EP_WATER; // <-- your sensor endpoint
  bind_req->dst_endp = COORDINATOR_ENDPOINT;
  bind_req->cluster_id = ESP_ZB_ZCL_CLUSTER_ID_REL_HUMIDITY_MEASUREMENT;

  bind_req->dst_addr_mode = ESP_ZB_ZDO_BIND_DST_ADDR_MODE_64_BIT_EXTENDED;

  // Destination IEEE = coordinator IEEE (looked up by short)
  esp_zb_ieee_address_by_short(COORDINATOR_SHORT_ADDR, bind_req->dst_address_u.addr_long);

  // Source IEEE = our IEEE
  esp_zb_get_long_address(bind_req->src_address);

  esp_zb_zdo_device_bind_req(bind_req, bind_cb, bind_req);
}

static void humidity_local_config_reporting(void) {
  esp_zb_zcl_config_report_cmd_t report_cmd = {0};

  report_cmd.zcl_basic_cmd.dst_addr_u.addr_short = esp_zb_get_short_address(); // local config
  report_cmd.zcl_basic_cmd.src_endpoint = EP_WATER;
  report_cmd.zcl_basic_cmd.dst_endpoint = EP_WATER;
  report_cmd.address_mode = ESP_ZB_APS_ADDR_MODE_16_ENDP_PRESENT;

  report_cmd.clusterID = ESP_ZB_ZCL_CLUSTER_ID_REL_HUMIDITY_MEASUREMENT;

  esp_zb_zcl_config_report_record_t rec = {
    .direction = ESP_ZB_ZCL_REPORT_DIRECTION_SEND,
    .attributeID = ESP_ZB_ZCL_ATTR_REL_HUMIDITY_MEASUREMENT_VALUE_ID,
    .attrType = ESP_ZB_ZCL_ATTR_TYPE_U16,

    // Pick sane defaults for a product:
    .min_interval = MIN_REPORTING_SEC, // no more often than every 5s
    .max_interval = MAX_REPORTING_SEC, // at least once a minute even if stable
    .reportable_change = &s_reportable_change, // 1.00% change
  };

  report_cmd.record_number = 1;
  report_cmd.record_field = &rec;

  esp_zb_lock_acquire(portMAX_DELAY);
  esp_zb_zcl_config_report_cmd_req(&report_cmd);
  esp_zb_lock_release();

  ESP_LOGI(TAG, "Local reporting configured");
}

/* -----------------------------
 * Zigbee: device model creation
 * ----------------------------- */

static esp_zb_cluster_list_t *create_onoff_light_clusters(bool initial_on) {
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

  return cluster_list;
}

static void zigbee_create_endpoints_and_register(void) {
  esp_zb_ep_list_t *ep_list = esp_zb_ep_list_create();

  /* Endpoint config templates */
  esp_zb_endpoint_config_t pump_ep_cfg = {
    .app_profile_id = ESP_ZB_AF_HA_PROFILE_ID,
    .app_device_id = ESP_ZB_ZCL_CLUSTER_ID_ON_OFF_SWITCH_CONFIG, /* Reuse HA On/Off Light for pump switch */
    .app_device_version = 0,
  };

  /* Pump 1 */
  {
    esp_zb_endpoint_config_t ep_cfg = pump_ep_cfg;
    ep_cfg.endpoint = EP_PUMP1;
    esp_zb_cluster_list_t *clusters = create_onoff_light_clusters(false);
    esp_zb_ep_list_add_ep(ep_list, clusters, ep_cfg);
  }
  /* Pump 2 */
  {
    esp_zb_endpoint_config_t ep_cfg = pump_ep_cfg;
    ep_cfg.endpoint = EP_PUMP2;
    esp_zb_cluster_list_t *clusters = create_onoff_light_clusters(false);
    esp_zb_ep_list_add_ep(ep_list, clusters, ep_cfg);
  }
  /* Pump 3 */
  {
    esp_zb_endpoint_config_t ep_cfg = pump_ep_cfg;
    ep_cfg.endpoint = EP_PUMP3;
    esp_zb_cluster_list_t *clusters = create_onoff_light_clusters(false);
    esp_zb_ep_list_add_ep(ep_list, clusters, ep_cfg);
  }

  /* Water level endpoint (custom attribute device) */
  {
    esp_zb_endpoint_config_t ep_cfg = {
      .endpoint = EP_WATER,
      .app_profile_id = ESP_ZB_AF_HA_PROFILE_ID,
      .app_device_id = ESP_ZB_ZCL_CLUSTER_ID_REL_HUMIDITY_MEASUREMENT,
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
          pump_set_by_endpoint(endpoint, on);

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
static void zb_retry_commissioning(uint8_t param) {
  (void) param;
  esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_NETWORK_STEERING);
}

void esp_zb_app_signal_handler(esp_zb_app_signal_t *signal_s) {
  /* esp-zigbee-lib 1.6.x pattern */
  esp_zb_app_signal_type_t sig =
      (signal_s && signal_s->p_app_signal)
        ? *(esp_zb_app_signal_type_t *) signal_s->p_app_signal
        : ESP_ZB_ZDO_SIGNAL_SKIP_STARTUP;

  esp_err_t status = signal_s ? signal_s->esp_err_status : ESP_FAIL;

  switch (sig) {
    case ESP_ZB_ZDO_SIGNAL_SKIP_STARTUP:
      ESP_LOGI(TAG, "Zigbee stack initialized, start commissioning (steering)");
      esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_NETWORK_STEERING);
      break;

    case ESP_ZB_BDB_SIGNAL_DEVICE_FIRST_START:
    case ESP_ZB_BDB_SIGNAL_DEVICE_REBOOT:
      if (status == ESP_OK) {
        esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_NETWORK_STEERING);
      } else {
        esp_zb_scheduler_alarm(zb_retry_commissioning, 0, 3000);
      }
      break;

    case ESP_ZB_BDB_SIGNAL_STEERING:
      if (status == ESP_OK) {
        ESP_LOGI(TAG, "Joined network OK (PAN: 0x%04hx, short: 0x%04hx)",
                 esp_zb_get_pan_id(), esp_zb_get_short_address());
        xEventGroupSetBits(s_zb_events, ZB_JOINED_BIT);
        humidity_bind_to_coordinator();
        start_release_timer_ms(120000);
      } else {
        ESP_LOGW(TAG, "Steering failed (%s). Retrying...", esp_err_to_name(status));
        /* schedule retry (see wrapper in section 3) */
        esp_zb_scheduler_alarm(zb_retry_commissioning, 0, 3000);
      }
      break;

    case ESP_ZB_COMMON_SIGNAL_CAN_SLEEP:
      EventBits_t bits = xEventGroupGetBits(s_zb_events);
      if (bits & ZB_JOINED_BIT) {
        // ESP_LOGI(TAG, "Zigbee sleeping...?");
        esp_zb_sleep_now();
      }
      break;

    case ESP_ZB_ZDO_SIGNAL_LEAVE:
      ESP_LOGW(TAG, "Left network");
      xEventGroupClearBits(s_zb_events, ZB_JOINED_BIT);
      esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_NETWORK_STEERING);
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

  /* Zigbee stack configuration */
  esp_zb_cfg_t zb_cfg = {
    .esp_zb_role = ESP_ZB_DEVICE_TYPE_ED, /* or ROUTER if you prefer */
    .install_code_policy = false,
    .nwk_cfg.zed_cfg = {
      .ed_timeout = ESP_ZB_ED_AGING_TIMEOUT_1024MIN,
      .keep_alive = MIN_REPORTING_SEC * 1000,
    },
  };

  esp_zb_sleep_enable(true);
  esp_zb_set_rx_on_when_idle(false);
  esp_zb_init(&zb_cfg);
  // esp_zb_zcl_analog_input_init_server();
  esp_zb_zcl_rel_humidity_measurement_init_server();
  zigbee_create_endpoints_and_register();
  esp_zb_core_action_handler_register(zb_action_handler);

  // esp_zb_nvram_erase_at_start(true); // erase NVRAM
  esp_zb_start(false);
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
    const struct WaterLevel* wl = tof_update();
    s_water_level_pct_x100 = (uint16_t) wl->percent * 100; // convert to 100ths

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

    vTaskDelay(pdMS_TO_TICKS(MIN_REPORTING_SEC * 1000));
  }
}

// static void water_attr_update_cb(uint8_t param)
// {
//   (void)param;
//
//   // dummy value
//   static uint16_t pct_x100 = 5000; // 50.00%
//   pct_x100 += 100;                // +1%
//   if (pct_x100 > 10000) pct_x100 = 0;
//
//   ESP_LOGI(TAG, "Water attribute update called %d%%", pct_x100);
//
//   s_water_level_pct_x100 = pct_x100;
//
//   esp_zb_lock_acquire(portMAX_DELAY);
//   esp_zb_zcl_set_attribute_val(
//       EP_WATER,
//       ESP_ZB_ZCL_CLUSTER_ID_REL_HUMIDITY_MEASUREMENT,
//       ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
//       ESP_ZB_ZCL_ATTR_REL_HUMIDITY_MEASUREMENT_VALUE_ID,
//       &s_water_level_pct_x100,
//       false
//   );
//   esp_zb_lock_release();
//
//   // schedule next update (e.g., 5 minutes)
//   esp_zb_scheduler_alarm(water_attr_update_cb, 0, MIN_REPORTING_SEC * 1000);
// }
/* -----------------------------
 * app_main
 * ----------------------------- */

void zb_start(void) {
  pm_lock_hold_for_join();
  s_zb_events = xEventGroupCreate();
  pumps_gpio_init();
  xTaskCreate(zigbee_task, "zigbee_task", 8192, NULL, 5, NULL);
  xTaskCreate(tof_task, "tof_task", 4096, NULL, 4, NULL);
}
