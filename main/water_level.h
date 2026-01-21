//
// Created by Zane Claes on 1/21/26.
//

#ifndef AUTO_PLANT_WATERER_WATER_LEVEL_H
#define AUTO_PLANT_WATERER_WATER_LEVEL_H

/* Water endpoint will expose Level Control cluster (0x0008) */
#define WATER_LEVEL_MIN_MM   150   // <-- set this: mm when tank is "100%" full (closest sensor distance)
#define WATER_LEVEL_MAX_MM   600   // <-- set this: mm when tank is "0%" empty (farthest distance)
#include <stdint.h>

#define I2C_PORT      I2C_NUM_0
#define I2C_SDA_GPIO  (GPIO_NUM_6) // SDA = GPIO6 (XIAO pin labeled SDA / D6)
#define I2C_SCL_GPIO  (GPIO_NUM_7) // SCL = GPIO7 (XIAO pin labeled SCL / D7)
#define I2C_FREQ_HZ   (400000)
#define TOF_I2C_ADDR  (0x29)

#endif //AUTO_PLANT_WATERER_WATER_LEVEL_H