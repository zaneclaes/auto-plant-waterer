//
// Created by Zane Claes on 1/26/26.
//

#ifndef AUTO_PLANT_WATERER_COORD_H
#define AUTO_PLANT_WATERER_COORD_H

#define SHARED_ADC_UNIT      ADC_UNIT_1

#include "esp_err.h"
#include "hal/adc_types.h"

#ifdef __cplusplus
extern "C" {
#endif

void shared_start();
adc_channel_t adc_start(int pin);
esp_err_t adc_shared_read(adc_channel_t ch, int* raw);
esp_err_t adc_read_avg(adc_channel_t ch, int* out);
void set_led(bool on);
void on_joined();

#ifdef __cplusplus
}
#endif

#endif //AUTO_PLANT_WATERER_COORD_H