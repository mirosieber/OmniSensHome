#ifndef SENSOR_H
#define SENSOR_H

#include "Arduino.h"
#include "OPT300x.h"
#include "Wire.h"
#include "Zigbee.h"
#include "configLoader.h"

extern ZigbeeCore Zigbee;
extern app_config_t *config;
extern ZigbeeIlluminanceSensor zbLuxSensor;
extern ZigbeeDimmableLight zbRgbLight;

void printError(String text, OPT300x_ErrorCode error);
void printResult(String text, OPT300x_S result);
void configureSensor();
void lux_sensor_value_update(void *arg);

#endif
