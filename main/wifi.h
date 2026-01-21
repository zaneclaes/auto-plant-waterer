//
// Created by Zane Claes on 1/21/26.
//

#ifndef AUTO_PLANT_WATERER_WIFI_H
#define AUTO_PLANT_WATERER_WIFI_H

// Station connect behavior
#define STA_MAX_RETRIES    8
#define STA_CONNECT_TIMEOUT_MS 15000

void wifi_start(void);

#endif //AUTO_PLANT_WATERER_WIFI_H