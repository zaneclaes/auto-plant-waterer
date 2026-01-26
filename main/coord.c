//
// Created by Zane Claes on 1/26/26.
//

#include "coord.h"

#include "pumps.h"
#include "soil.h"

void on_joined() {
  pumps_start();
  soil_start();
}
