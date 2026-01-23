//
// Created by Zane Claes on 1/21/26.
//

#ifndef AUTO_PLANT_WATERER_WATER_PUMPS_H
#define AUTO_PLANT_WATERER_WATER_PUMPS_H

#include <stdint.h>
#include "rom/secure_boot.h"

#define PUMP1_GPIO  (GPIO_NUM_1)
#define PUMP2_GPIO  (GPIO_NUM_2)
#define PUMP3_GPIO  (GPIO_NUM_21)

void pumps_start(void);
void pump_set(uint8_t idx, bool on);

#endif //AUTO_PLANT_WATERER_WATER_PUMPS_H