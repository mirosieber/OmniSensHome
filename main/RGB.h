#ifndef RGB_H
#define RGB_H

#include "configLoader.h"
#include <Arduino.h>
// RGB LED brightness control
static uint8_t rgb_brightness = 100; // Default full brightness (0-255)
static uint8_t rgb_red = 0;
static uint8_t rgb_green = 0;
static uint8_t rgb_blue = 0;

void setRgbLedColor(app_config_t *config, uint8_t red, uint8_t green,
                    uint8_t blue);
void setRgbLedBrightness(app_config_t *config, uint8_t brightness);
void onRgbLightChange(app_config_t *config, bool state, uint8_t level);

#endif