//
// Created by Zane Claes on 1/21/26.
//

#ifndef AUTO_PLANT_WATERER_ZB_H
#define AUTO_PLANT_WATERER_ZB_H
#include "rom/secure_boot.h"

void zb_start(void);
void zb_report(bool sensors, bool battery);
void zb_on_ready();

#endif //AUTO_PLANT_WATERER_ZB_H