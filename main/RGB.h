#ifndef RGB_H
#define RGB_H

#include "configLoader.h"
#include <Arduino.h>

void setRgbLedColor(app_config_t *config, uint8_t red, uint8_t green,
                    uint8_t blue);
void setRgbLedBrightness(app_config_t *config, uint8_t brightness);
void onRgbLightChange(app_config_t *config, bool state, uint8_t level);

#endif