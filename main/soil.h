//
// Created by Zane Claes on 1/26/26.
//

#ifndef AUTO_PLANT_WATERER_SOIL_H
#define AUTO_PLANT_WATERER_SOIL_H
#include <stdint.h>

struct SoilLevel {
  int raw;            // i.e., 3700
  int mv;             // i.e., 3700
  uint8_t percent;    // 0-100 range
};

void soil_start(void);
const struct SoilLevel* soil_get_level(uint8_t idx);

#endif //AUTO_PLANT_WATERER_SOIL_H
