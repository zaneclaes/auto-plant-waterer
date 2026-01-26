//
// Created by Zane Claes on 1/26/26.
//

#ifndef AUTO_PLANT_WATERER_COORD_H
#define AUTO_PLANT_WATERER_COORD_H

#define SHARED_ADC_UNIT      ADC_UNIT_1

#include "esp_err.h"
#include "hal/adc_types.h"

void shared_start();
adc_channel_t adc_start(int pin);
esp_err_t adc_shared_read(adc_channel_t ch, int* raw);
void on_joined();

#endif //AUTO_PLANT_WATERER_COORD_H