//
// Created by Zane Claes on 1/21/26.
//

#ifndef AUTO_PLANT_WATERER_WATER_LEVEL_H
#define AUTO_PLANT_WATERER_WATER_LEVEL_H

#include <stdint.h>
#include "esp_err.h"
#ifdef __cplusplus
extern "C" {
#endif

/* Water endpoint will expose Level Control cluster (0x0008) */
#define WATER_LEVEL_MIN_MM   150   // <-- set this: mm when tank is "100%" full (closest sensor distance)
#define WATER_LEVEL_MAX_MM   600   // <-- set this: mm when tank is "0%" empty (farthest distance)

#define TOF_I2C_PORT      I2C_NUM_0
#define TOF_I2C_SDA_GPIO  (GPIO_NUM_22) // SDA = GPIO22 (XIAO pin labeled SDA / D6)
#define TOF_I2C_SCL_GPIO  (GPIO_NUM_23) // SCL = GPIO23 (XIAO pin labeled SCL / D7)
#define TOF_I2C_FREQ_HZ   (400000)
#define TOF_I2C_ADDR      (0x29)

struct WaterLevel {
  uint16_t mm;
  uint16_t mm_clamped;
  uint8_t  percent;
};

esp_err_t tof_start(void);
const struct WaterLevel* tof_update(void);

#ifdef __cplusplus
}
#endif

#endif //AUTO_PLANT_WATERER_WATER_LEVEL_H