#ifndef SENSOR_H
#define SENSOR_H

#include "Arduino.h"
#include "MTS4x.h"
#include "OPT300x.h"
#include "Wire.h"
#include <AHTxx.h>

extern "C" {
#include "configLoader.h"
}

MTS4X MTS4Z = MTS4X();
OPT300x opt3004;

void initSensors(app_config_t *config);

#endif
