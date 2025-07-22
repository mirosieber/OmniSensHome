#include "Arduino.h"
#include "Zigbee.h"
#include "esp_zigbee_core.h"
#include "esp_zigbee_ota.h"
#include <AHTxx.h>

#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "configLoader.h"

/* Zigbee OTA configuration */
#define OTA_UPGRADE_RUNNING_FILE_VERSION                                       \
  0x01010100 // Increment this value when the running image is updated
#define OTA_UPGRADE_DOWNLOADED_FILE_VERSION                                    \
  0x01010101 // Increment this value when the downloaded image is updated
#define OTA_UPGRADE_HW_VERSION                                                 \
  0x0101 // The hardware version, this can be used to differentiate between
         // different hardware versions

static const char *TAG = "main";

extern "C" void app_main(void) {
  // Initialize Arduino runtime
  initArduino();
  // Setup serial communication
  Serial.begin(115200);
  while (!Serial) {
    ; // Wait for Serial to initialize (for USB CDC targets)
  }

  // Allocate config on heap to avoid stack overflow
  app_config_t *config = (app_config_t *)malloc(sizeof(app_config_t));
  if (config == NULL) {
    ESP_LOGE(TAG, "Failed to allocate memory for config");
    return;
  }

  ESP_LOGI(TAG, "Loading configuration from /spiffs/config.json");
  if (!(config_load(config) == ESP_OK)) {
    ESP_LOGE(TAG, "Failed to load configuration");
    free(config);
    return;
  }
  ESP_LOGI(TAG, "Device Model: %s", config->device.model);

  // Jeder Endpoint hat eine eindeutige Nummer (zwischen 1 und 240)
  ZigbeeRangeExtender zbRangeExtender = ZigbeeRangeExtender(1);
  ZigbeeIlluminanceSensor zbLuxSensor = ZigbeeIlluminanceSensor(2);
  ZigbeeTempSensor zbTempSensor = ZigbeeTempSensor(3);
  ZigbeeTempSensor zbTempHumiditySensor = ZigbeeTempSensor(4);
  ZigbeeCarbonDioxideSensor zbAirQuality = ZigbeeCarbonDioxideSensor(5);
  ZigbeeAnalog zbDBSensor = ZigbeeAnalog(6);
  ZigbeeOccupancySensor zbOccupancySensor = ZigbeeOccupancySensor(7);
  ZigbeeContactSwitch zbContactSwitches[4] = {
      ZigbeeContactSwitch(8), ZigbeeContactSwitch(9), ZigbeeContactSwitch(10),
      ZigbeeContactSwitch(11)};
  ZigbeeLight zbRelays[4] = {ZigbeeLight(12), ZigbeeLight(13), ZigbeeLight(14),
                             ZigbeeLight(15)};

  if (config->rgb_led.enabled) {
    pinMode(config->rgb_led.red_pin, OUTPUT);
    pinMode(config->rgb_led.green_pin, OUTPUT);
    pinMode(config->rgb_led.blue_pin, OUTPUT);
    digitalWrite(config->rgb_led.red_pin, LOW);
    digitalWrite(config->rgb_led.green_pin, LOW);
    digitalWrite(config->rgb_led.blue_pin, LOW);
  }
  // Init button for factory reset
  pinMode(config->factory_reset_pin, INPUT_PULLUP);

  // set Zigbee device name and model

  zbTempSensor.setManufacturerAndModel(config->device.manufacturer,
                                       config->device.model);
  if (strcmp(config->device.power_supply, "battery") == 0) {
    zbTempSensor.setPowerSource(ZB_POWER_SOURCE_BATTERY);
  } else {
    zbTempSensor.setPowerSource(ZB_POWER_SOURCE_MAINS);
  }

  // Set callback function for relays change
  // zbRelays[0].onLightChange(setRelays);

  // Add OTA client to the light bulb
  zbTempSensor.addOTAClient(OTA_UPGRADE_RUNNING_FILE_VERSION,
                            OTA_UPGRADE_DOWNLOADED_FILE_VERSION,
                            OTA_UPGRADE_HW_VERSION);
  // Add endpoint to Zigbee Core
  if (strcmp(config->device.type, "Router") == 0) {
    Zigbee.addEndpoint(&zbRangeExtender);
  }

  for (uint8_t i = 0; config->sensors[i].type[0] != '\0'; i++) {
    if (config->sensors[i].enabled) {
      if (strcmp(config->sensors[i].type, "OPT3004") == 0) {
        // Initialize OPT3004 sensor
        Zigbee.addEndpoint(&zbLuxSensor);
      } else if (strcmp(config->sensors[i].type, "MTS4Z") == 0) {
        // Initialize MTS4Z sensor
        Zigbee.addEndpoint(&zbTempSensor);
      } else if (strcmp(config->sensors[i].type, "AHT21") == 0) {
        // Initialize AHT21 sensor
        Zigbee.addEndpoint(&zbTempHumiditySensor);
      } else if (strcmp(config->sensors[i].type, "ENS160") == 0) {
        // Initialize ENS160 sensor
        Zigbee.addEndpoint(&zbAirQuality);
      } else if (strcmp(config->sensors[i].type, "INMP441") == 0) {
        // Initialize INMP441 sensor
        Zigbee.addEndpoint(&zbDBSensor);
      } else if (strcmp(config->sensors[i].type, "HLK-LD2412") == 0) {
        // Initialize HLK-LD2412 sensor
        Zigbee.addEndpoint(&zbOccupancySensor);
      } else if (strcmp(config->sensors[i].type, "Bluetooth") == 0) {
        // Initialize Bluetooth sensor
      } else {
        ESP_LOGE(TAG, "Sensor type %s not recognized\n",
                 config->sensors[i].type);
      }
    }
  }
  for (uint8_t i = 0; i < 4; i++) {
    if (config->switches[i].enabled) {
      // Use the initialized array
      Zigbee.addEndpoint(&zbContactSwitches[i]);
    }
  }

  for (uint8_t i = 0; i < 4; i++) {
    if (config->relays[i].enabled) {
      Zigbee.addEndpoint(&zbRelays[i]);
    }
  }

  // When all EPs are registered, start Zigbee.
  if (strcmp(config->device.type, "Router") == 0) {
    if (!Zigbee.begin(ZIGBEE_ROUTER)) {
      Serial.println("Zigbee failed to start!");
      Serial.println("Rebooting...");
      ESP.restart();
    }
  } else if (strcmp(config->device.type, "EndDevice") == 0) {
    if (!Zigbee.begin(ZIGBEE_END_DEVICE)) {
      Serial.println("Zigbee failed to start!");
      Serial.println("Rebooting...");
      ESP.restart();
    }
  }

  Serial.println("Connecting to network");
  while (!Zigbee.connected()) {
    Serial.print(".");
    delay(100);
  }
  Serial.println();

  // Start Zigbee OTA client query, first request is within a minute and the
  // next requests are sent every hour automatically
  zbTempSensor.requestOTAUpdate();
  uint8_t button = config->factory_reset_pin;
  while (true) {
    // Checking button for factory reset
    if (digitalRead(button) == LOW) { // Push button pressed
      // Key debounce handling
      delay(100);
      int startTime = millis();
      while (digitalRead(button) == LOW) {
        delay(50);
        if ((millis() - startTime) > 3000) {
          // If key pressed for more than 3secs, factory reset Zigbee and
          // reboot
          Serial.println("Resetting Zigbee to factory and rebooting in 1s.");
          delay(1000);
          Zigbee.factoryReset();
        }
      }
      // Toggle light by pressing the button
      // zbLight.setLight(!zbLight.getLightState());
    }
    delay(100);
  }

  // Clean up (though this code will never be reached due to infinite loop)
  free(config);
}
