#ifndef AUTO_PLANT_WATERER_BAT_H
#define AUTO_PLANT_WATERER_BAT_H

#include <stdint.h>

#define VOLTAGE_MIN 3.3f
#define VOLTAGE_MAX 4.2f

#define VBAT_R1 100000.0f // resistor 1 = 100 kOhm
#define VBAT_R2 100000.0f // resistor 2 = 100 kOhm
#define VBAT_DIVIDER_RATIO ((VBAT_R1 + VBAT_R2) / VBAT_R2)

struct BatteryLevel {
  float voltage;      // i.e., 3.7v
  uint8_t percent;    // 0-100 range
};

void battery_start(void);
void enable_power_management(void);
const struct BatteryLevel* battery_update(void);

#endif //AUTO_PLANT_WATERER_BAT_H