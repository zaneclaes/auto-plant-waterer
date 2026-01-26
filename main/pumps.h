//
// Created by Zane Claes on 1/21/26.
//

#ifndef AUTO_PLANT_WATERER_WATER_PUMPS_H
#define AUTO_PLANT_WATERER_WATER_PUMPS_H

#include <stdint.h>
#include "rom/secure_boot.h"

#define PIN_PUMP1  (GPIO_NUM_18)
#define PIN_PUMP2  (GPIO_NUM_20)
#define PIN_PUMP3  (GPIO_NUM_19)

void pumps_start(void);
void pump_set(uint8_t idx, bool on);

#endif //AUTO_PLANT_WATERER_WATER_PUMPS_H