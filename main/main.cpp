#include "Arduino.h"
#include "Zigbee.h"
#include "esp_zigbee_core.h"
#include "esp_zigbee_ota.h"

#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "configLoader.h"

#include "MTS4x.h"
#include "OPT300x.h"
#include "SparkFun_ENS160.h"
#include "Wire.h"
#include <AHTxx.h>

/* Zigbee OTA configuration */
#define OTA_UPGRADE_RUNNING_FILE_VERSION                                       \
  0x01010100 // Increment this value when the running image is updated
#define OTA_UPGRADE_DOWNLOADED_FILE_VERSION                                    \
  0x01010101 // Increment this value when the downloaded image is updated
#define OTA_UPGRADE_HW_VERSION                                                 \
  0x0101 // The hardware version, this can be used to differentiate between
         // different hardware versions

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

MTS4X MTS4Z = MTS4X();
OPT300x opt3004;
AHTxx aht21(AHTXX_ADDRESS_X38, AHTXX_I2C_SENSOR::AHT2x_SENSOR);
SparkFun_ENS160 ens160;

void printError(String text, OPT300x_ErrorCode error) {
  Serial.print(text);
  Serial.print(": [ERROR] Code #");
  Serial.println(error);
}

void printResult(String text, OPT300x_S result) {
  if (result.error == NO_ERROR) {
    Serial.print(text);
    Serial.print(": ");
    Serial.print(result.lux);
    Serial.println(" lux");
  } else {
    printError(text, result.error);
  }
}

void configureSensor() {
  OPT300x_Config newConfig;

  newConfig.RangeNumber = B1100;
  newConfig.ConvertionTime = B0;
  newConfig.Latch = B1;
  newConfig.ModeOfConversionOperation = B11;

  OPT300x_ErrorCode errorConfig = opt3004.writeConfig(newConfig);
  if (errorConfig != NO_ERROR)
    printError("OPT300x configuration", errorConfig);
  else {
    OPT300x_Config sensorConfig = opt3004.readConfig();
    Serial.println("OPT300x Current Config:");
    Serial.println("------------------------------");

    Serial.print("Conversion ready (R):");
    Serial.println(sensorConfig.ConversionReady, HEX);

    Serial.print("Conversion time (R/W):");
    Serial.println(sensorConfig.ConvertionTime, HEX);

    Serial.print("Fault count field (R/W):");
    Serial.println(sensorConfig.FaultCount, HEX);

    Serial.print("Flag high field (R-only):");
    Serial.println(sensorConfig.FlagHigh, HEX);

    Serial.print("Flag low field (R-only):");
    Serial.println(sensorConfig.FlagLow, HEX);

    Serial.print("Latch field (R/W):");
    Serial.println(sensorConfig.Latch, HEX);

    Serial.print("Mask exponent field (R/W):");
    Serial.println(sensorConfig.MaskExponent, HEX);

    Serial.print("Mode of conversion operation (R/W):");
    Serial.println(sensorConfig.ModeOfConversionOperation, HEX);

    Serial.print("Polarity field (R/W):");
    Serial.println(sensorConfig.Polarity, HEX);

    Serial.print("Overflow flag (R-only):");
    Serial.println(sensorConfig.OverflowFlag, HEX);

    Serial.print("Range number (R/W):");
    Serial.println(sensorConfig.RangeNumber, HEX);

    Serial.println("------------------------------");
  }
}

/************************ Temperature sensor task ****************************/
static void temp_sensor_value_update(void *arg) {
  for (;;) {
    MTS4Z.startSingleMessurement();
    float temperature = MTS4Z.readTemperature(true);
    Serial.printf("[Temp Sensor] Temperature: %.2f°C\r\n", temperature);
    zbTempSensor.setTemperature(temperature);
    delay(1000);
  }
}

/********************* Lux sensor task **************************/
static void lux_sensor_value_update(void *arg) {
  const char *TAG = "Lux Task";
  for (;;) {
    OPT300x_S result = opt3004.readResult();
    float lux = result.lux;
    Serial.printf("[Lux Sensor] Lux: %.2f\r\n", lux);
    if (lux > 0) {
      float raw = 10000.0 * log10(lux);
      zbLuxSensor.setIlluminance((uint16_t)raw);
    } else {
      zbLuxSensor.setIlluminance(0);
    }
    delay(1000);
  }
}

/************ Temperature and humidity sensor task***************/
static void temp_humidity_sensor_value_update(void *arg) {
  const char *TAG = "Temp/Humidity Task";
  for (;;) {
    float ahtTemp =
        aht21.readTemperature(); // read 6-bytes via I2C, takes 80 milliseconds
    float ahtHumidity =
        aht21.readHumidity(AHTXX_USE_READ_DATA); // use data from temperature
                                                 // read, takes 0 milliseconds
    Serial.printf(
        "[Temp/Humidity Sensor] Temperature: %.2f°C, Humidity: %.2f%%\r\n",
        ahtTemp, ahtHumidity);
    zbTempHumiditySensor.setTemperature(ahtTemp);
    zbTempHumiditySensor.setHumidity(ahtHumidity);

    // ens160 compensation
    ens160.setTempCompensationCelsius(ahtTemp);
    ens160.setRHCompensationFloat(ahtHumidity);
    uint16_t eco2 = ens160.getECO2();
    uint16_t tvoc = ens160.getTVOC();
    Serial.printf("[ENS160] eCO2: %d ppm, TVOC: %d ppb\r\n", eco2, tvoc);
    zbAirQuality.setCarbonDioxide(eco2);
    delay(1000);
  }
}

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
        Wire.begin(config->i2c.sda, config->i2c.scl);
        opt3004.begin(config->sensors[i].i2c_address);
        configureSensor();
      } else if (strcmp(config->sensors[i].type, "MTS4Z") == 0) {
        // Initialize MTS4Z sensor
        Zigbee.addEndpoint(&zbTempSensor);
        MTS4Z.begin(config->i2c.sda, config->i2c.scl);
        delay(10);
        MTS4Z.setConfig(MPS_8Hz, AVG_32, true);
      } else if (strcmp(config->sensors[i].type, "AHT21") == 0) {
        // Initialize AHT21 sensor
        zbTempHumiditySensor.addHumiditySensor(0, 100,
                                               1); // min = 0%, max = 100%,
                                                   // tolerance = 1%
        Zigbee.addEndpoint(&zbTempHumiditySensor);
        aht21.begin(config->i2c.sda, config->i2c.scl);
      } else if (strcmp(config->sensors[i].type, "ENS160") == 0) {
        // Initialize ENS160 sensor
        Zigbee.addEndpoint(&zbAirQuality);
        ens160.begin(config->sensors[i].i2c_address);
        ens160.setOperatingMode(SFE_ENS160_STANDARD);
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

  // temporär
  xTaskCreate(lux_sensor_value_update, "lux_sensor_update", 2048, NULL, 10,
              NULL);
  zbLuxSensor.setReporting(1, 0, 1000); // delta = 1000 raw illuminance
  xTaskCreate(temp_sensor_value_update, "temp_sensor_update", 2048, NULL, 10,
              NULL);
  zbTempSensor.setReporting(1, 0, 1); // delta = 0.1°C
  xTaskCreate(temp_humidity_sensor_value_update, "temp_humidity_sensor_update",
              2048, NULL, 10, NULL);
  zbTempHumiditySensor.setReporting(1, 0, 1); // delta = 0.1°C
  zbAirQuality.setReporting(1, 0, 100);       // delta = 100 ppm eCO2

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
