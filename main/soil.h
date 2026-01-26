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

// n.b., must be ADC pins!
#define PIN_SOIL1     (GPIO_NUM_0)  // D0
#define ADC_SOIL1     (ADC_CHANNEL_0)

#define PIN_SOIL2     (GPIO_NUM_1)  // D1
#define ADC_SOIL2     (ADC_CHANNEL_1)

#define PIN_SOIL3     (GPIO_NUM_2)  // D2
#define ADC_SOIL3     (ADC_CHANNEL_2)

#define PIN_SOIL_ON   (GPIO_NUM_21) // D3

void soil_start(void);
const struct SoilLevel* soil_get_level(uint8_t idx);

#endif //AUTO_PLANT_WATERER_SOIL_H
