/**
 * @file VL53L0X.h
 * @author Ryotaro Onuki (kerikun11+github@gmail.com)
 * @brief C++ Library for VL53L0X as an ESP-IDF component
 * @copyright Copyright (c) 2018 Ryotaro Onuki
 */
#pragma once

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

#include "driver/gpio.h"
#include "driver/i2c_master.h"

#include "vl53l0x_api.h"
#include "vl53l0x_def.h"
#include "vl53l0x_platform.h"

#include "esp_log.h"

static constexpr uint8_t VL53L0X_I2C_ADDRESS_DEFAULT = 0x29;

/**
 * @brief VL53L0X class
 *
 */
class VL53L0X {
public:
  VL53L0X(i2c_port_t i2c_port = I2C_NUM_0, gpio_num_t gpio_xshut = GPIO_NUM_MAX,
          gpio_num_t gpio_gpio1 = GPIO_NUM_MAX)
      : i2c_port(i2c_port), gpio_xshut(gpio_xshut), gpio_gpio1(gpio_gpio1) {
    vSemaphoreCreateBinary(xSemaphore);
  }

  bool init() {
    /* gpio init */
    if (gpio_xshut != GPIO_NUM_MAX) {
      gpio_set_direction(gpio_xshut, GPIO_MODE_OUTPUT);
      gpio_set_level(gpio_xshut, 1);
    }
    if (gpio_gpio1 != GPIO_NUM_MAX) {
      ESP_ERROR_CHECK(gpio_set_direction(gpio_gpio1, GPIO_MODE_INPUT));
      ESP_ERROR_CHECK(gpio_set_pull_mode(gpio_gpio1, GPIO_PULLUP_ONLY));
      ESP_ERROR_CHECK(gpio_pullup_en(gpio_gpio1));
      ESP_ERROR_CHECK(gpio_set_intr_type(gpio_gpio1, GPIO_INTR_POSEDGE));
      ESP_ERROR_CHECK(gpio_install_isr_service(0));
      ESP_ERROR_CHECK(gpio_isr_handler_add(gpio_gpio1, gpio1_isr, this));
      ESP_ERROR_CHECK(gpio_intr_enable(gpio_gpio1));
    }
    /* device init */
    vl53l0x_dev.i2c_port_num = i2c_port;
    vl53l0x_dev.i2c_address = VL53L0X_I2C_ADDRESS_DEFAULT;
    reset();
    if (init_vl53l0x(&vl53l0x_dev) != VL53L0X_ERROR_NONE)
      return false;
    if (VL53L0X_ERROR_NONE !=
        VL53L0X_SetGpioConfig(&vl53l0x_dev, 0,
                              VL53L0X_DEVICEMODE_SINGLE_RANGING,
                              VL53L0X_GPIOFUNCTIONALITY_NEW_MEASURE_READY,
                              VL53L0X_INTERRUPTPOLARITY_LOW))
      return false;
    if (!setTimingBudget(33000))
      return false;
    return true;
  }
  bool reset() {
    if (!softwareReset())
      return hardwareReset();
    return true;
  }
  bool hardwareReset() {
    if (gpio_xshut == GPIO_NUM_MAX)
      return false;
    gpio_set_level(gpio_xshut, 0);
    vTaskDelay(10 / portTICK_PERIOD_MS);
    gpio_set_level(gpio_xshut, 1);
    vTaskDelay(10 / portTICK_PERIOD_MS);
    return true;
  }
  bool softwareReset() {
    VL53L0X_Error status = VL53L0X_ResetDevice(&vl53l0x_dev);
    if (status != VL53L0X_ERROR_NONE) {
      print_pal_error(status, "VL53L0X_ResetDevice");
      return false;
    }
    return true;
  }
  /**
   * @brief Set the I2C address of the VL53L0X
   *
   * @param new_address right-aligned address
   */
  bool setDeviceAddress(uint8_t new_address) {
    // ST API expects left-aligned address
    VL53L0X_Error status = VL53L0X_SetDeviceAddress(&vl53l0x_dev, new_address << 1);
    if (status != VL53L0X_ERROR_NONE) {
      print_pal_error(status, "VL53L0X_SetDeviceAddress");
      return false;
    }

    // Update legacy field
    vl53l0x_dev.i2c_address = new_address;

    // IMPORTANT: update the new-driver device handle to use the new 7-bit address
    if (i2c_bus) {
      // Remove old device handle and add a new one with the updated address.
      // (No "set address" API; re-add device is the supported approach.)
      if (vl53l0x_dev.i2c_dev) {
        ESP_ERROR_CHECK(i2c_master_bus_rm_device(vl53l0x_dev.i2c_dev));
        vl53l0x_dev.i2c_dev = nullptr;
      }

      i2c_device_config_t dev_cfg = {};
      dev_cfg.dev_addr_length = I2C_ADDR_BIT_LEN_7;
      dev_cfg.device_address = new_address;     // right-aligned 7-bit
      dev_cfg.scl_speed_hz = 400000;            // or store your freq and reuse it

      ESP_ERROR_CHECK(i2c_master_bus_add_device(i2c_bus, &dev_cfg, &vl53l0x_dev.i2c_dev));
    }

    return true;
  }

  bool read(uint16_t *pRangeMilliMeter) {
    if (gpio_gpio1 != GPIO_NUM_MAX)
      return readSingleWithInterrupt(pRangeMilliMeter);
    return readSingleWithPolling(pRangeMilliMeter);
  }
  bool readSingleWithPolling(uint16_t *pRangeMilliMeter) {
    VL53L0X_RangingMeasurementData_t MeasurementData;
    VL53L0X_Error status =
        VL53L0X_PerformSingleRangingMeasurement(&vl53l0x_dev, &MeasurementData);
    if (status != VL53L0X_ERROR_NONE) {
      print_pal_error(status, "VL53L0X_PerformSingleRangingMeasurement");
      return false;
    }
    *pRangeMilliMeter = MeasurementData.RangeMilliMeter;
    if (MeasurementData.RangeStatus != 0)
      return false;
    return true;
  }
  bool readSingleWithDelay(uint16_t *pRangeMilliMeter) {
    TickType_t xTicksToWait =
        TimingBudgetMicroSeconds / 1000 / portTICK_PERIOD_MS;
    VL53L0X_Error status;
    // set mode
    status =
        VL53L0X_SetDeviceMode(&vl53l0x_dev, VL53L0X_DEVICEMODE_SINGLE_RANGING);
    if (status != VL53L0X_ERROR_NONE) {
      print_pal_error(status, "VL53L0X_SetDeviceMode");
      return false;
    }
    // start measurement
    status = VL53L0X_StartMeasurement(&vl53l0x_dev);
    if (status != VL53L0X_ERROR_NONE) {
      print_pal_error(status, "VL53L0X_StartMeasurement");
      return false;
    }
    // wait
    vTaskDelay(xTicksToWait);
    // get data
    VL53L0X_RangingMeasurementData_t MeasurementData;
    status = VL53L0X_GetRangingMeasurementData(&vl53l0x_dev, &MeasurementData);
    if (status != VL53L0X_ERROR_NONE) {
      print_pal_error(status, "VL53L0X_GetRangingMeasurementData");
      return false;
    }
    *pRangeMilliMeter = MeasurementData.RangeMilliMeter;
    if (MeasurementData.RangeStatus != 0)
      return false;
    // clear interrupt
    VL53L0X_ClearInterruptMask(&vl53l0x_dev, 0);
    if (status != VL53L0X_ERROR_NONE)
      return false;
    return true;
  }
  bool readSingleWithInterrupt(uint16_t *pRangeMilliMeter) {
    if (gpio_gpio1 == GPIO_NUM_MAX)
      return false;
    VL53L0X_Error status;
    // set mode
    status =
        VL53L0X_SetDeviceMode(&vl53l0x_dev, VL53L0X_DEVICEMODE_SINGLE_RANGING);
    if (status != VL53L0X_ERROR_NONE) {
      print_pal_error(status, "VL53L0X_SetDeviceMode");
      return false;
    }
    // clear semphr
    xSemaphoreTake(xSemaphore, 0);
    // start measurement
    status = VL53L0X_StartMeasurement(&vl53l0x_dev);
    if (status != VL53L0X_ERROR_NONE) {
      print_pal_error(status, "VL53L0X_StartMeasurement");
      return false;
    }
    // wait for interrupt
    xSemaphoreTake(xSemaphore, 1000 / portMAX_DELAY);
    // get data
    VL53L0X_RangingMeasurementData_t MeasurementData;
    status = VL53L0X_GetRangingMeasurementData(&vl53l0x_dev, &MeasurementData);
    if (status != VL53L0X_ERROR_NONE) {
      print_pal_error(status, "VL53L0X_GetRangingMeasurementData");
      return false;
    }
    *pRangeMilliMeter = MeasurementData.RangeMilliMeter;
    if (MeasurementData.RangeStatus != 0)
      return false;
    // clear interrupt
    VL53L0X_ClearInterruptMask(&vl53l0x_dev, 0);
    if (status != VL53L0X_ERROR_NONE)
      return false;
    return true;
  }

  void i2cMasterInit(gpio_num_t pin_sda = GPIO_NUM_21,
                   gpio_num_t pin_scl = GPIO_NUM_22,
                   uint32_t freq = 400000) {
    // If already initialized, do nothing (or you can re-init by deleting first)
    if (i2c_bus && vl53l0x_dev.i2c_dev) return;

    // Create I2C master bus
    i2c_master_bus_config_t bus_cfg = {};
    bus_cfg.i2c_port = i2c_port;
    bus_cfg.sda_io_num = pin_sda;
    bus_cfg.scl_io_num = pin_scl;
    bus_cfg.clk_source = I2C_CLK_SRC_DEFAULT;
    bus_cfg.glitch_ignore_cnt = 7;
    bus_cfg.flags.enable_internal_pullup = true; // uses internal pullups; ok if you also have external

    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_cfg, &i2c_bus));

    // Add the VL53L0X device to the bus
    i2c_device_config_t dev_cfg = {};
    dev_cfg.dev_addr_length = I2C_ADDR_BIT_LEN_7;
    dev_cfg.device_address = VL53L0X_I2C_ADDRESS_DEFAULT; // 0x29 right-aligned
    dev_cfg.scl_speed_hz = freq;

    ESP_ERROR_CHECK(i2c_master_bus_add_device(i2c_bus, &dev_cfg, &vl53l0x_dev.i2c_dev));

    // Keep legacy fields updated if other code uses them
    vl53l0x_dev.i2c_port_num = i2c_port;
    vl53l0x_dev.i2c_address = VL53L0X_I2C_ADDRESS_DEFAULT;
  }

  bool setTimingBudget(uint32_t TimingBudgetMicroSeconds) {
    VL53L0X_Error status = VL53L0X_SetMeasurementTimingBudgetMicroSeconds(
        &vl53l0x_dev, TimingBudgetMicroSeconds);
    if (status != VL53L0X_ERROR_NONE) {
      print_pal_error(status, "VL53L0X_SetMeasurementTimingBudgetMicroSeconds");
      return false;
    }
    this->TimingBudgetMicroSeconds = TimingBudgetMicroSeconds;
    return true;
  }

protected:
  i2c_master_bus_handle_t i2c_bus = nullptr;

  static constexpr const char *TAG = "VL53L0X";
  i2c_port_t i2c_port;
  gpio_num_t gpio_xshut;
  gpio_num_t gpio_gpio1;
  VL53L0X_Dev_t vl53l0x_dev;
  volatile SemaphoreHandle_t xSemaphore = NULL;
  int32_t TimingBudgetMicroSeconds;

  static void IRAM_ATTR gpio1_isr(void *arg) {
    VL53L0X *obj = static_cast<VL53L0X *>(arg);
    xSemaphoreGiveFromISR(obj->xSemaphore, NULL);
  }
  static VL53L0X_Error print_pal_error(VL53L0X_Error status,
                                       const char *method) {
    char buf[VL53L0X_MAX_STRING_LENGTH];
    VL53L0X_GetPalErrorString(status, buf);
    ESP_LOGE(TAG, "%s API status: %i : %s\n", method, status, buf);
    return status;
  }
  static VL53L0X_Error init_vl53l0x(VL53L0X_Dev_t *pDevice) {
    VL53L0X_Error status;
    uint8_t isApertureSpads;
    uint8_t PhaseCal;
    uint32_t refSpadCount;
    uint8_t VhvSettings;
    // Device Initialization (~40ms)
    status = VL53L0X_DataInit(pDevice);
    if (status != VL53L0X_ERROR_NONE)
      return print_pal_error(status, "VL53L0X_DataInit");
    status = VL53L0X_StaticInit(pDevice);
    if (status != VL53L0X_ERROR_NONE)
      return print_pal_error(status, "VL53L0X_StaticInit");
    // SPADs calibration (~10ms)
    status = VL53L0X_PerformRefSpadManagement(pDevice, &refSpadCount,
                                              &isApertureSpads);
    ESP_LOGI(TAG, "refSpadCount = %" PRIu32 ", isApertureSpads = %" PRIu8 "\n",
             refSpadCount, isApertureSpads);
    if (status != VL53L0X_ERROR_NONE)
      return print_pal_error(status, "VL53L0X_PerformRefSpadManagement");
    // Temperature calibration (~40ms)
    status = VL53L0X_PerformRefCalibration(pDevice, &VhvSettings, &PhaseCal);
    if (status != VL53L0X_ERROR_NONE)
      return print_pal_error(status, "VL53L0X_PerformRefCalibration");
    // Setup in single ranging mode
    status = VL53L0X_SetDeviceMode(pDevice, VL53L0X_DEVICEMODE_SINGLE_RANGING);
    if (status != VL53L0X_ERROR_NONE)
      return print_pal_error(status, "VL53L0X_SetDeviceMode");
    // end
    return status;
  }
};
