//
// Created by Zane Claes on 1/26/26.
//

#include "coord.h"

#include "bat.h"
#include "cfg.h"
#include "esp_log.h"
#include "esp_pm.h"
#include "esp_timer.h"
#include "pumps.h"
#include "soil.h"
#include "tof.h"
#include "zb.h"
#include "driver/gpio.h"
#include "esp_adc/adc_oneshot.h"
#include "soc/gpio_num.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_timer.h"

static const char *TAG = "coord";

#define PIN_LED     GPIO_NUM_15

static adc_oneshot_unit_handle_t adc_handle;

void set_led(bool on) {
  gpio_set_level(PIN_LED, on ? 0 : 1);
}
void dump_timers_once(void) {
  esp_timer_dump(stdout);
}
void dump_tasks_once(void) {
  // Enable CONFIG_FREERTOS_USE_TRACE_FACILITY and CONFIG_FREERTOS_GENERATE_RUN_TIME_STATS if you want more detail
  char buf[2048];
  vTaskList(buf);
  // Print it once, then stop (one print won’t ruin everything)
  printf("Task          State Prio Stack Num\n%s\n", buf);
}

/* -----------------------------
 * Events / signaling for joining
 * ----------------------------- */

// static EventGroupHandle_t s_zb_events;
#define ZB_JOINED_BIT     BIT0
#define ZB_JOINING_BIT    BIT1
#define ZB_READY_BIT      BIT2
#define ZB_REPORTING_BIT  BIT3

static bool s_joined = false;
static bool s_joining = false;
static bool s_ready = false;
static bool s_reporting = false;

bool is_joined(void) { return s_joined; } // (xEventGroupGetBits(s_zb_events) & ZB_JOINED_BIT) != 0; }
bool is_joining(void) { return s_joining; }//  (xEventGroupGetBits(s_zb_events) & ZB_JOINING_BIT) != 0; }
bool is_ready(void) { return s_ready; }// (xEventGroupGetBits(s_zb_events) & ZB_READY_BIT) != 0; }
bool is_reporting(void) { return s_reporting; } // (xEventGroupGetBits(s_zb_events) & ZB_REPORTING_BIT) != 0; }

void set_joined(bool joined) {
  s_joined = joined;
  // if (joined) xEventGroupSetBits(s_zb_events, ZB_JOINED_BIT);
  // else xEventGroupClearBits(s_zb_events, ZB_JOINED_BIT);
}

void set_joining(bool joining) {
  s_joining = joining;
  // if (joining) xEventGroupSetBits(s_zb_events, ZB_JOINING_BIT);
  // else xEventGroupClearBits(s_zb_events, ZB_JOINING_BIT);
}

void set_ready(bool ready) {
  s_ready = ready;
  if (ready) {
    // xEventGroupSetBits(s_zb_events, ZB_READY_BIT);
    zb_on_ready();
  }
  // else xEventGroupClearBits(s_zb_events, ZB_READY_BIT);
}

void wait_for_join(void) {
  while (!is_joined()) { vTaskDelay(pdMS_TO_TICKS(1000)); }
  // xEventGroupWaitBits(s_zb_events, ZB_JOINED_BIT, pdFALSE, pdTRUE, portMAX_DELAY);
}

/* -----------------------------
 * LOCK; prevent power management while joining...
 * ----------------------------- */

static esp_pm_lock_handle_t s_pm_no_ls_lock;
// static esp_pm_lock_handle_t s_pm_cpu_max_lock;
static esp_timer_handle_t s_release_timer;

void pm_lock(void) {
  ESP_LOGI(TAG, "Pausing power management...");
  if (!s_pm_no_ls_lock) {
    ESP_ERROR_CHECK(esp_pm_lock_create(ESP_PM_NO_LIGHT_SLEEP, 0, "zb_join", &s_pm_no_ls_lock));
  }
  // if (!s_pm_cpu_max_lock) {
  //   ESP_ERROR_CHECK(esp_pm_lock_create(ESP_PM_CPU_FREQ_MAX, 0, "zb_join_cpu", &s_pm_cpu_max_lock));
  // }
  ESP_ERROR_CHECK(esp_pm_lock_acquire(s_pm_no_ls_lock));
  // ESP_ERROR_CHECK(esp_pm_lock_acquire(s_pm_cpu_max_lock));
}

static void pm_lock_release(void) {
  ESP_LOGI(TAG, "Resuming power management %d...", s_pm_no_ls_lock ? 1 : 0);
  if (s_pm_no_ls_lock) {
    esp_pm_lock_release(s_pm_no_ls_lock);
  }
  // if (s_pm_cpu_max_lock) esp_pm_lock_release(s_pm_cpu_max_lock);
}

static void pm_lock_release_after_join(void *arg) {
  if (!is_joined()) {
    ESP_LOGW(TAG, "Power management timer hit, but not joined");
    return;
  }
  pm_lock_release();
  // set_ready(true);
}

void pm_release_after_ms(uint32_t ms) {
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


static void timed_water(uint8_t idx, uint16_t sec) {
  ESP_LOGI(TAG, "Watering #%u for %u seconds...", idx, sec);
  pump_set(idx, true);
  vTaskDelay(pdMS_TO_TICKS(sec * 1000));
  pump_set(idx, false);
}

static void read_all_sensors(void) {
  uint8_t num_zones = get_num_zones();
  update_water_level();
  soil_enable();
  for (uint8_t i = 0; i < num_zones; i++) {
    update_soil_level(i);
  }
  soil_disable();

  for (uint8_t i = 0; i < num_zones; i++) {
    const struct SoilLevel* sl = get_soil_level(i);
    if (sl->percent < 10) {
      timed_water(i, 10);
    }
  }
  update_battery();
}

static void report_task(void *pv) {
  ESP_LOGI(TAG, "Starting reporting...");
  uint32_t sec_since_bat = 0;
  uint32_t sec_since_sensors = 0;
  uint32_t read_cnt = 0;
  while (is_joined()) {
    read_all_sensors();

    bool sensors = read_cnt <= REPORT_AVG_CNT || sec_since_sensors > BAT_REPORTING_MIN_SEC;
    bool battery = read_cnt <= REPORT_AVG_CNT || sec_since_bat > BAT_REPORTING_MIN_SEC;
    if (sensors || battery) zb_report(sensors, battery);
    if (sensors) sec_since_sensors = 0;
    if (battery) sec_since_bat = 0;

    read_cnt++;
    vTaskDelay(pdMS_TO_TICKS(REPORT_SLEEP_SEC * 1000));
    sec_since_bat += REPORT_SLEEP_SEC;
    sec_since_sensors += REPORT_SLEEP_SEC;
  }
  ESP_LOGI(TAG, "Reporting ended");
  vTaskDelete(NULL);
}

// static void battery_task(void* pv){
//   while (is_joined()) {
//     update_battery();
//     zb_report_battery();
//     vTaskDelay(pdMS_TO_TICKS(BAT_REPORTING_MIN_SEC * 1000));
//   }
//   ESP_LOGI(TAG, "Battery reporting ended");
//   vTaskDelete(NULL);
// }

void shared_start() {
  ESP_LOGI(TAG, "Shared Start...");
  // s_zb_events = xEventGroupCreate();
  gpio_config_t cfg = {
    .pin_bit_mask = 1ULL << PIN_LED,
    .mode = GPIO_MODE_OUTPUT,
    .pull_up_en = GPIO_PULLUP_DISABLE,
    .pull_down_en = GPIO_PULLDOWN_DISABLE,
    .intr_type = GPIO_INTR_DISABLE,
  };
  ESP_ERROR_CHECK(gpio_config(&cfg));
  set_led(true);

  adc_oneshot_unit_init_cfg_t init_cfg = {
    .unit_id = SHARED_ADC_UNIT,
    .ulp_mode = ADC_ULP_MODE_DISABLE,
  };
  ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_cfg, &adc_handle));
}

adc_channel_t adc_start(int pin) {
  adc_unit_t unit;
  adc_channel_t ch;
  ESP_ERROR_CHECK(adc_oneshot_io_to_channel(pin, &unit, &ch));
  if (unit != SHARED_ADC_UNIT) ESP_LOGW(TAG, "GPIO%d mapped to unexpected ADC unit %d", pin, (int)unit);

  adc_oneshot_chan_cfg_t cfg = {
    .atten = ADC_ATTEN_DB_12, // widest range; good for 0-3.3V-ish sensors
    .bitwidth = ADC_BITWIDTH_DEFAULT,
  };
  ESP_ERROR_CHECK(adc_oneshot_config_channel(adc_handle, ch, &cfg));
  return ch;
}

esp_err_t adc_shared_read(const adc_channel_t ch, int* raw) {
  return adc_oneshot_read(adc_handle, ch, raw);
}

static int s_moving_avg[10][REPORT_AVG_CNT-1] = {};
static int moving_avg(uint8_t channel, int value) {
  long sum = value;
  int cnt = 1;
  // ESP_LOGI(TAG, "Moving Avg #%u: %d %d %d %d %d", channel,
  //   s_moving_avg[channel][0], s_moving_avg[channel][1], s_moving_avg[channel][2],
  //   s_moving_avg[channel][3], value);
  for (uint8_t i=0; i<(REPORT_AVG_CNT-1); i++) {
    if (s_moving_avg[channel][i] != 0) {
      sum += s_moving_avg[channel][i];
      cnt++;
    }
    if (i<(REPORT_AVG_CNT-2))  s_moving_avg[channel][i] = s_moving_avg[channel][i+1];
  }
  s_moving_avg[channel][REPORT_AVG_CNT-2] = value;
  return sum / cnt;
}

#define NUM_SAMPLES        8
#define SAMPLE_DELAY_MS    5
esp_err_t adc_read_avg(const adc_channel_t ch, int* out) {
  int sum = 0;
  for (int i = 0; i < NUM_SAMPLES; i++) {
    int raw = 0;
    esp_err_t err = adc_shared_read(ch, &raw);
    if (err != ESP_OK) return err;
    sum += raw;
    vTaskDelay(pdMS_TO_TICKS(SAMPLE_DELAY_MS));
  }
  *out = moving_avg(ch, sum / NUM_SAMPLES);
  return ESP_OK;
}

void on_joined() {
  if (get_cfg_flag(CFG_FLAG_RESET)) {
    set_cfg_flag(CFG_FLAG_RESET, false);
    cfg_save();
  }
  pm_release_after_ms(90000);
  set_joined(true);
  set_joining(false);
  xTaskCreate(report_task, "report_task", 0x1000, NULL, 4, NULL);
  set_led(false);
  // xTaskCreate(battery_task, "battery_task", 0x1000, NULL, 3, NULL);
}

void on_left() {
  set_joined(false);
  set_joining(false);
  set_ready(false);
  set_led(true);
}