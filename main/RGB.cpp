#include "RGB.h"

// RGB LED state variables (file-local)
static uint8_t rgb_brightness = 100; // Default brightness (0-255)
static uint8_t rgb_red = 0;
static uint8_t rgb_green = 0;
static uint8_t rgb_blue = 0;

const char *TAG = "RGB"; // Logging tag for ESP_LOGI

void setRgbLedColor(app_config_t *config, uint8_t red, uint8_t green,
                    uint8_t blue) {
  if (config->rgb_led.enabled) {
    // Store the raw RGB values
    rgb_red = red;
    rgb_green = green;
    rgb_blue = blue;

    // Apply brightness scaling using the level from ZigbeeColorDimmableLight
    uint8_t scaled_red = (red * rgb_brightness) / 255;
    uint8_t scaled_green = (green * rgb_brightness) / 255;
    uint8_t scaled_blue = (blue * rgb_brightness) / 255;

    analogWrite(config->rgb_led.red_pin, scaled_red);
    analogWrite(config->rgb_led.green_pin, scaled_green);
    analogWrite(config->rgb_led.blue_pin, scaled_blue);
  }
}

void setRgbLedBrightness(app_config_t *config, uint8_t brightness) {
  rgb_brightness = brightness;
  // Reapply current colors with new brightness
  setRgbLedColor(config, rgb_red, rgb_green, rgb_blue);
  ESP_LOGI(TAG, "RGB LED brightness set to %d", brightness);
}

/********************* RGB LED Zigbee callbacks **************************/
void onRgbLightChange(app_config_t *config, bool state, uint8_t level) {
  // Always store the brightness level
  rgb_brightness = level;

  if (state) {
    // Light is ON - Set RGB LED with current color and brightness
    setRgbLedColor(config, rgb_red, rgb_green, rgb_blue);
    ESP_LOGI(TAG, "RGB LED state: ON (R:%d, G:%d, B:%d, Level:%d)", rgb_red,
             rgb_green, rgb_blue, level);
  } else {
    // Light is OFF - Turn off all LEDs
    if (config->rgb_led.enabled) {
      analogWrite(config->rgb_led.red_pin, 0);
      analogWrite(config->rgb_led.green_pin, 0);
      analogWrite(config->rgb_led.blue_pin, 0);
    }
    ESP_LOGI(TAG, "RGB LED turned OFF");
  }
}