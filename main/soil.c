//
// Created by Zane Claes on 1/26/26.
//

#include "soil.h"
#include "cfg.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_err.h"
#include "driver/gpio.h"

#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"

static const char *TAG = "soil";

// Configure 3 ADC channels (derived from GPIO)
const gpio_num_t gpios[3] = { PIN_SOIL1, PIN_SOIL2, PIN_SOIL3 };
const adc_channel_t adcs[3] = { ADC_SOIL1, ADC_SOIL2, ADC_SOIL3 };

struct SoilLevel levels[3] = {
  {
    .raw = 0,
    .mv = 0,
    .percent = 0,
  },   {
    .raw = 0,
    .mv = 0,
    .percent = 0,
  },   {
    .raw = 0,
    .mv = 0,
    .percent = 0,
  }
};

#define NUM_SAMPLES        32
#define SAMPLE_DELAY_MS    5

static int g_dry_mv = 2800;   // reading in air / dry soil (mV)
static int g_wet_mv = 1400;   // reading in water / saturated soil (mV)

static adc_oneshot_unit_handle_t adc_handle;
static adc_cali_handle_t cali_handle;
static bool cali_enabled = false;

#define SOIL_ADC_UNIT      ADC_UNIT_1

void soil_start(void) {
  // Power gate pin (D3)
  gpio_config_t pwr = {
    .pin_bit_mask = 1ULL << PIN_SOIL_ON,
    .mode = GPIO_MODE_OUTPUT,
    .pull_up_en = GPIO_PULLUP_DISABLE,
    .pull_down_en = GPIO_PULLDOWN_DISABLE,
    .intr_type = GPIO_INTR_DISABLE,
  };
  ESP_ERROR_CHECK(gpio_config(&pwr));
  gpio_set_level(PIN_SOIL_ON, 0); // start OFF

  // ADC unit
  adc_oneshot_unit_init_cfg_t init_cfg = {
    .unit_id = ADC_UNIT_1,
    .ulp_mode = ADC_ULP_MODE_DISABLE,
  };
  ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_cfg, &adc_handle));

  for (int i = 0; i < NUM_ZONES; i++) {
    adc_unit_t unit;
    adc_channel_t ch;
    ESP_ERROR_CHECK(adc_oneshot_io_to_channel(gpios[i], &unit, &ch));
    // Sanity: should all be ADC_UNIT_1 on XIAO C6 for GPIO0/1/2
    if (unit != ADC_UNIT_1) ESP_LOGW(TAG, "GPIO%d mapped to unexpected ADC unit %d", (int)gpios[i], (int)unit);

    adc_oneshot_chan_cfg_t cfg = {
      .atten = ADC_ATTEN_DB_12, // widest range; good for 0-3.3V-ish sensors
      .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc_handle, ch, &cfg));

    ESP_LOGI(TAG, "Soil %d: GPIO%d -> ADC_UNIT_%d CH_%d", i, (int)gpios[i], (int)unit, (int)ch);
  }

  // ---- Optional calibration (recommended) ----
  // Not all chips / configs support all schemes; we try line fitting first.
#if ADC_CALI_SCHEME_LINE_FITTING_SUPPORTED
  adc_cali_line_fitting_config_t cali_cfg = {
    .unit_id = SOIL_ADC_UNIT,
    .atten = ADC_ATTEN_DB_11,
    .bitwidth = ADC_BITWIDTH_DEFAULT,
};
  if (adc_cali_create_scheme_line_fitting(&cali_cfg, &cali_handle) == ESP_OK) {
    cali_enabled = true;
    ESP_LOGI(TAG, "ADC calibration: line fitting enabled");
  } else {
    ESP_LOGW(TAG, "ADC calibration: line fitting not available");
  }
#elif ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
  adc_cali_curve_fitting_config_t cali_cfg = {
    .unit_id = SOIL_ADC_UNIT,
    .atten = ADC_ATTEN_DB_12,
    .bitwidth = ADC_BITWIDTH_DEFAULT,
};
  if (adc_cali_create_scheme_curve_fitting(&cali_cfg, &cali_handle) == ESP_OK) {
    cali_enabled = true;
    ESP_LOGI(TAG, "ADC calibration: curve fitting enabled");
  } else {
    ESP_LOGW(TAG, "ADC calibration: curve fitting not available");
  }
#else
  ESP_LOGW(TAG, "ADC calibration: no supported scheme in this IDF build");
#endif
}

static esp_err_t read_soil_raw_avg(uint8_t idx, int *raw_out){
  if (idx > NUM_SAMPLES) return ESP_FAIL;
  adc_channel_t ch = adcs[idx];
  int sum = 0;
  for (int i = 0; i < NUM_SAMPLES; i++) {
    int raw = 0;
    esp_err_t err = adc_oneshot_read(adc_handle, ch, &raw);
    if (err != ESP_OK) return err;
    sum += raw;
    vTaskDelay(pdMS_TO_TICKS(SAMPLE_DELAY_MS));
  }
  *raw_out = sum / NUM_SAMPLES;
  return ESP_OK;
}

static int raw_to_millivolts(int raw, int *mv_out){
  if (cali_enabled) {
    int mv = 0;
    esp_err_t err = adc_cali_raw_to_voltage(cali_handle, raw, &mv);
    if (err == ESP_OK) {
      *mv_out = mv;
      return ESP_OK;
    }
    // If calibration fails for any reason, fall back to raw.
    ESP_LOGW(TAG, "adc_cali_raw_to_voltage failed (%s), using raw", esp_err_to_name(err));
  }
  *mv_out = raw;
  return ESP_FAIL;
}

// Maps mV to "moisture %" using your wet/dry calibration.
// Many capacitive sensors output LOWER voltage when wetter (common),
// but yours might be inverted—swap endpoints if needed.
static int mv_to_moisture_percent(int mv){
  // Clamp & map: 0% = dry, 100% = wet
  int dry = g_dry_mv;
  int wet = g_wet_mv;

  // Avoid divide-by-zero
  if (dry == wet) return 0;

  // If wet < dry (typical), mapping still works.
  float pct = (float)(dry - mv) * 100.0f / (float)(dry - wet);

  if (pct < 0.0f) pct = 0.0f;
  if (pct > 100.0f) pct = 100.0f;
  return (int)(pct + 0.5f);
}

const struct SoilLevel* soil_get_level(uint8_t idx) {
  if (idx > NUM_SAMPLES) return NULL;
  esp_err_t err = read_soil_raw_avg(idx, &levels[idx].raw);
  if (err != ESP_OK) {
    ESP_LOGW(TAG, "Failed to read soil: %s", esp_err_to_name(err));
    return NULL;
  }

  if (raw_to_millivolts(levels[idx].raw, &levels[idx].mv) == ESP_OK) {
    levels[idx].percent = mv_to_moisture_percent(levels[idx].mv);
    ESP_LOGI(TAG, "Soil: raw=%d, mv=%d, moisture=%d%% (dry=%dmV wet=%dmV)",
             levels[idx].raw, levels[idx].mv, levels[idx].percent, g_dry_mv, g_wet_mv);
  } else {
    // If no mV, still print raw so you can calibrate using raw endpoints instead.
    ESP_LOGI(TAG, "Soil: raw=%d (no mV calibration). Tune endpoints with raw if needed.", levels[idx].raw);
  }

  return &levels[idx];
}