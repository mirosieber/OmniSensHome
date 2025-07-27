#include "Arduino.h"
#include "Zigbee.h"
#include "esp_zigbee_attribute.h"
#include "esp_zigbee_cluster.h"
#include "esp_zigbee_core.h"
#include "esp_zigbee_endpoint.h"
#include "esp_zigbee_ota.h"

#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "configLoader.h"

#include "DbSensor.h"
#include "MTS4x.h"
#include "OPT300x.h"
#include "SparkFun_ENS160.h"
#include "Wire.h"
#include "ld2412.h"
#include "speaker.h"
#include "uart_config.h"
#include <AHTxx.h>

static const char *TAG = "main";

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
ZigbeeCarbonDioxideSensor zbeCo2Sensor = ZigbeeCarbonDioxideSensor(5);
ZigbeeCarbonDioxideSensor zbTVOCSensor =
    ZigbeeCarbonDioxideSensor(16); // TVOC sensor endpoint
ZigbeeAnalog zbDBSensor = ZigbeeAnalog(6);
ZigbeeBinary zbOccupancySensor = ZigbeeBinary(7);
// Alternative: Use binary sensors instead of IAS contact switches (more
// compatible)
ZigbeeBinary zbBinarySensors[4] = {ZigbeeBinary(8), ZigbeeBinary(9),
                                   ZigbeeBinary(10), ZigbeeBinary(11)};
uint8_t switchNr =
    0; // Variable to keep track of the number of contact switches
ZigbeeLight zbRelays[4] = {ZigbeeLight(12), ZigbeeLight(13), ZigbeeLight(14),
                           ZigbeeLight(15)};
ZigbeeDimmableLight zbRgbLight =
    ZigbeeDimmableLight(17); // RGB LED dimmable light endpoint

// Custom LD2412 Bluetooth Control using standard ZigbeeLight (On/Off cluster)
ZigbeeLight zbLD2412BluetoothControl = ZigbeeLight(18);

MTS4X MTS4Z = MTS4X();
OPT300x opt3004;
AHTxx aht21(AHTXX_ADDRESS_X38, AHTXX_I2C_SENSOR::AHT2x_SENSOR);
SparkFun_ENS160 ens160;
Speaker speaker; // Speaker instance

app_config_t *config;

// RGB LED brightness control
static uint8_t rgb_brightness = 255; // Default full brightness (0-255)
static uint8_t rgb_red = 0;
static uint8_t rgb_green = 0;
static uint8_t rgb_blue = 0;

// LD2412 sensor variables
static bool ld2412_initialized = false;
static bool ld2412_bluetooth_enabled = true; // Track current Bluetooth state

// LED color update timing control
static unsigned long last_led_update_time = 0;
static const unsigned long LED_UPDATE_INTERVAL =
    30000; // 30 seconds in milliseconds

// External reference to system_queue defined in ld2412_local component
extern QueueHandle_t system_queue;

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

  newConfig.RangeNumber = 0b1100;
  newConfig.ConvertionTime = 0b0;
  newConfig.Latch = 0b1;
  newConfig.ModeOfConversionOperation = 0b11;

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

void setRgbLedColor(uint8_t red, uint8_t green, uint8_t blue) {
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

void setRgbLedBrightness(uint8_t brightness) {
  rgb_brightness = brightness;
  // Reapply current colors with new brightness
  setRgbLedColor(rgb_red, rgb_green, rgb_blue);
  ESP_LOGI(TAG, "RGB LED brightness set to %d", brightness);
}

/********************* RGB LED Zigbee callbacks **************************/
void onRgbLightChange(bool state, uint8_t level) {
  // Always store the brightness level
  rgb_brightness = level;

  if (state) {
    // Light is ON - Set RGB LED with current color and brightness
    setRgbLedColor(rgb_red, rgb_green, rgb_blue);
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

/************* LD2412 Bluetooth Control Functions**************/
void onLD2412BluetoothControl(bool bluetooth_enable) {
  if (!ld2412_initialized) {
    ESP_LOGW(TAG, "LD2412 not initialized, ignoring Bluetooth control command");
    return;
  }

  ESP_LOGI(TAG, "LD2412 Bluetooth control request: %s",
           bluetooth_enable ? "ENABLE" : "DISABLE");

  // Only change if state is different
  if (bluetooth_enable != ld2412_bluetooth_enabled) {
    ld2412_bluetooth_enabled = bluetooth_enable;

    // Control LD2412 Bluetooth based on new state
    if (ld2412_bluetooth_enabled) {
      ESP_LOGI(TAG, "Enabling LD2412 Bluetooth via Zigbee command");
      enable_bluetooth();
    } else {
      ESP_LOGI(TAG, "Disabling LD2412 Bluetooth via Zigbee command");
      disable_bluetooth();
    }

    ESP_LOGI(TAG, "LD2412 Bluetooth state changed to: %s",
             ld2412_bluetooth_enabled ? "ENABLED" : "DISABLED");
  } else {
    ESP_LOGI(TAG, "LD2412 Bluetooth state unchanged: %s",
             ld2412_bluetooth_enabled ? "ENABLED" : "DISABLED");
  }
}

/********************* Relay control functions **************************/
void setRelay0(bool value) {
  if (config->relays[0].enabled) {
    digitalWrite(config->relays[0].pin, value);
    ESP_LOGI(TAG, "Relay 0 (%s) set to %s", config->relays[0].name,
             value ? "ON" : "OFF");
  }
}

void setRelay1(bool value) {
  if (config->relays[1].enabled) {
    digitalWrite(config->relays[1].pin, value);
    ESP_LOGI(TAG, "Relay 1 (%s) set to %s", config->relays[1].name,
             value ? "ON" : "OFF");
  }
}

void setRelay2(bool value) {
  if (config->relays[2].enabled) {
    digitalWrite(config->relays[2].pin, value);
    ESP_LOGI(TAG, "Relay 2 (%s) set to %s", config->relays[2].name,
             value ? "ON" : "OFF");
  }
}

void setRelay3(bool value) {
  if (config->relays[3].enabled) {
    digitalWrite(config->relays[3].pin, value);
    ESP_LOGI(TAG, "Relay 3 (%s) set to %s", config->relays[3].name,
             value ? "ON" : "OFF");
  }
}

/************************ Temperature sensor task ****************************/
static void temp_sensor_value_update(void *arg) {
  // Wait for Zigbee network to be ready
  while (!Zigbee.connected()) {
    delay(100);
  }
  MTS4Z.begin(config->i2c.sda, config->i2c.scl, MEASURE_SINGLE);
  delay(10);
  // frequenz hier egal, da single mode aktiv
  MTS4Z.setConfig(MPS_1Hz, AVG_32, true);

  // Additional stabilization delay
  delay(5000);

  ESP_LOGI(TAG, "MTS4Z temperature sensor task starting...");

  for (;;) {
    MTS4Z.startSingleMessurement();
    float temperature = MTS4Z.readTemperature(true);

    // Validate temperature reading
    if (temperature > -40.0 && temperature < 85.0) {
      // Serial.printf("[Temp Sensor] Temperature: %.2f°C\r\n", temperature);
      zbTempSensor.setTemperature(temperature);
    } else {
      ESP_LOGW(TAG, "Invalid temperature reading: %.2f°C", temperature);
    }
    delay(5000);
  }
}

/********************* Lux sensor task **************************/
static void lux_sensor_value_update(void *arg) {
  // Wait for Zigbee network to be ready
  while (!Zigbee.connected()) {
    delay(100);
  }

  // Find the correct sensor configuration index for OPT3004
  int sensor_index = -1;
  for (uint8_t i = 0; config->sensors[i].type[0] != '\0'; i++) {
    if (strcmp(config->sensors[i].type, "OPT3004") == 0) {
      sensor_index = i;
      break;
    }
  }

  if (sensor_index == -1) {
    ESP_LOGE(TAG, "OPT3004 sensor configuration not found");
    vTaskDelete(NULL);
    return;
  }

  Wire.begin(config->i2c.sda, config->i2c.scl);
  opt3004.begin(config->sensors[sensor_index].i2c_address);
  configureSensor();

  // Additional stabilization delay
  delay(3000);

  ESP_LOGI(TAG, "OPT3004 lux sensor task starting...");

  for (;;) {
    OPT300x_S result = opt3004.readResult();
    float lux = result.lux;

    if (result.error == NO_ERROR && lux >= 0) {
      // Serial.printf("[Lux Sensor] Lux: %.2f\r\n", lux);
      if (lux > 0) {
        float raw = 10000.0 * log10(lux);
        zbLuxSensor.setIlluminance((uint16_t)raw);
      } else {
        zbLuxSensor.setIlluminance(0);
      }
    } else {
      ESP_LOGW(TAG, "Error reading lux sensor or invalid value: %.2f", lux);
    }

    delay(500);
  }
}

/************ Temperature and humidity and air quality sensor
 * task***************/
static void temp_humidity_sensor_value_update(void *arg) {
  // Wait for Zigbee network to be ready
  while (!Zigbee.connected()) {
    delay(100);
  }

  // Find the correct sensor configuration index for ENS160
  int sensor_index = -1;
  for (uint8_t i = 0; config->sensors[i].type[0] != '\0'; i++) {
    if (strcmp(config->sensors[i].type, "ENS160") == 0) {
      sensor_index = i;
      break;
    }
  }

  if (sensor_index == -1) {
    ESP_LOGE(TAG, "ENS160 sensor configuration not found");
    vTaskDelete(NULL);
    return;
  }

  aht21.begin(config->i2c.sda, config->i2c.scl);
  delay(10);
  ens160.begin(config->sensors[sensor_index].i2c_address);
  ens160.setOperatingMode(SFE_ENS160_STANDARD);

  // Additional stabilization delay
  delay(7000);

  ESP_LOGI(TAG, "AHT21 temperature/humidity sensor task starting...");

  for (;;) {
    float ahtTemp =
        aht21.readTemperature(); // read 6-bytes via I2C, takes 80 milliseconds
    float ahtHumidity =
        aht21.readHumidity(AHTXX_USE_READ_DATA); // use data from temperature
                                                 // read, takes 0 milliseconds

    // Validate readings
    if (ahtTemp > -40.0 && ahtTemp < 85.0 && ahtHumidity >= 0.0 &&
        ahtHumidity <= 100.0) {
      // Serial.printf(
      //     "[Temp/Humidity Sensor] Temperature: %.2f°C, Humidity: %.2f%%\r\n",
      //     ahtTemp, ahtHumidity);
      zbTempHumiditySensor.setTemperature(ahtTemp);
      zbTempHumiditySensor.setHumidity(ahtHumidity);

      // ens160 compensation
      ens160.setTempCompensationCelsius(ahtTemp);
      ens160.setRHCompensationFloat(ahtHumidity);
      uint16_t eco2 = ens160.getECO2();
      uint16_t tvoc = ens160.getTVOC();

      // Validate ENS160 readings
      if (eco2 >= 400 && eco2 <= 65000) { // Typical eCO2 range
        // Serial.printf("[ENS160] eCO2: %d ppm, TVOC: %d ppb\r\n", eco2, tvoc);
        zbeCo2Sensor.setCarbonDioxide(eco2);
        // Check if enough time has passed since last LED update (30 seconds)
        unsigned long current_time = millis();
        if (config->rgb_led.enabled && zbRgbLight.getLightState() &&
            (current_time - last_led_update_time >= LED_UPDATE_INTERVAL)) {

          // Set RGB LED color and brightness based on eCO2 level - only if
          // light is currently ON and 30 seconds have passed
          if (eco2 < 600 && tvoc < 65) {
            // Good air quality - cyan with low brightness
            rgb_red = 137;
            rgb_green = 254;
            rgb_blue = 255;
          } else if (eco2 < 800 && tvoc < 220) {
            // Acceptable air quality - light green with medium-low brightness
            rgb_red = 50;
            rgb_green = 238;
            rgb_blue = 50;
          } else if (eco2 < 1000 && tvoc < 650) {
            // Moderate air quality - gold with medium brightness
            rgb_red = 255;
            rgb_green = 215;
            rgb_blue = 0;
          } else if (eco2 < 1500 && tvoc < 2200) {
            // Poor air quality - light red with high brightness
            rgb_red = 255;
            rgb_green = 80;
            rgb_blue = 80;
          } else {
            // Very poor air quality - red with full brightness
            rgb_red = 255;
            rgb_green = 0;
            rgb_blue = 0;
          }
          setRgbLedColor(rgb_red, rgb_green, rgb_blue);
          last_led_update_time = current_time; // Update the last update time
        }

        // Validate TVOC readings (typical range 0-60000 ppb)
        if (tvoc <= 60000) {
          zbTVOCSensor.setCarbonDioxide(
              tvoc); // Using CO2 sensor for TVOC (ppb)
        } else {
          ESP_LOGW(TAG, "Invalid TVOC reading: %d ppb", tvoc);
        }
      } else {
        ESP_LOGW(TAG, "Invalid eCO2 reading: %d ppm", eco2);
      }
    } else {
      ESP_LOGW(TAG, "Invalid AHT21 readings - Temp: %.2f°C, Humidity: %.2f%%",
               ahtTemp, ahtHumidity);
    }

    delay(10000); // Increased delay to reduce sensor heating
  }
}

/*****************Db sensor task ****************/
static void db_sensor_value_update(void *param) {
  // Wait for Zigbee network to be ready
  while (!Zigbee.connected()) {
    delay(100);
  }

  // Additional stabilization delay
  delay(8000);

  ESP_LOGI(TAG, "INMP441 dB sensor task starting...");

  // Find the correct sensor configuration index for INMP441
  int sensor_index = -1;
  for (uint8_t i = 0; config->sensors[i].type[0] != '\0'; i++) {
    if (strcmp(config->sensors[i].type, "INMP441") == 0) {
      sensor_index = i;
      break;
    }
  }

  if (sensor_index == -1) {
    ESP_LOGE(TAG, "INMP441 sensor configuration not found");
    vTaskDelete(NULL);
    return;
  }
  zbDBSensor.setAnalogInputReporting(
      0, 30, 10); // report every 30 seconds if value changes by 10
  DbSensor dbSensor(static_cast<gpio_num_t>(config->sensors[sensor_index].sck),
                    static_cast<gpio_num_t>(config->sensors[sensor_index].ws),
                    static_cast<gpio_num_t>(config->sensors[sensor_index].sd));

  // Sensor initialisieren
  dbSensor.begin();

  for (;;) {
    float db = 40.0; // dbSensor.getCurrentDb();

    // Validate dB reading (typical range for INMP441)
    if (db >= 30.0 && db <= 130.0) {
      // Serial.printf("[dB Sensor] Sound level: %.2f dB\r\n", db);

      // Use the actual dB value directly since we configured it as a dB
      // sensor
      zbDBSensor.setAnalogInput(db);

      // Report the analog input value
      // zbDBSensor.reportAnalogInput();
    } else {
      ESP_LOGW(TAG, "Invalid dB reading: %.2f dB", db);
    }

    delay(100); // Update every 100 milliseconds
  }
}

// Simple contact switch state tracking (like Arduino example)
static bool contact_states[4] = {false, false, false, false};

/*****************contact switches task ****************/
static void contact_switches_task(void *arg) {
  ESP_LOGI(TAG, "=== Binary Sensor Monitoring Task ===");
  ESP_LOGI(TAG, "Using binary sensors (more compatible than IAS zones)");

  // Initialize contact states (like Arduino example)
  for (uint8_t i = 0; i < 4; i++) {
    if (config->switches[i].enabled) {
      contact_states[i] = (digitalRead(config->switches[i].pin) == HIGH);
      ESP_LOGI(TAG, "Binary sensor %d initialized: %s", i,
               contact_states[i] ? "on" : "off");

      // Set initial state for binary sensor
      zbBinarySensors[i].setBinaryInput(contact_states[i]);
      zbBinarySensors[i].reportBinaryInput(); // Report initial state
    }
  }

  ESP_LOGI(TAG, "Binary sensor monitoring active");

  for (;;) {
    // Simple polling like the Arduino example
    for (uint8_t i = 0; i < 4; i++) {
      if (config->switches[i].enabled) {
        // Check pin state (HIGH = on, LOW = off for binary sensors)
        bool current_pin_state = (digitalRead(config->switches[i].pin) == HIGH);

        // State change detection
        if (current_pin_state != contact_states[i]) {
          ESP_LOGI(TAG, "Binary sensor %d: %s -> %s", i,
                   contact_states[i] ? "on" : "off",
                   current_pin_state ? "on" : "off");

          // Update binary sensor if state changed
          zbBinarySensors[i].setBinaryInput(current_pin_state);
          bool report_result = zbBinarySensors[i].reportBinaryInput();

          if (report_result) {
            ESP_LOGI(TAG, "✓ Binary sensor %d reported successfully", i);
          } else {
            ESP_LOGW(TAG, "✗ Binary sensor %d report failed", i);
          }

          contact_states[i] = current_pin_state;
        }
      }
    }

    delay(50); // fast polling interval
  }
}

/*************Presence Detection Task ****************/
static void occupancy_sensor_value_update(void *arg) {
  // Wait for Zigbee network to be ready
  while (!Zigbee.connected()) {
    delay(100);
  }

  const char *TAG = "Occupancy Sensor Task";
  // ESP_LOGI(TAG, "=== LD2412 Occupancy Sensor Task Started ===");

  // Initialize UART buffers and configuration for LD2412 communication
  uart_buffer_init();
  uart_config();

  // The system_queue is created by uart_config(), so just mark as
  // initialized
  // ESP_LOGI(TAG, "LD2412 system queue created by uart_config()");
  ld2412_initialized = true;

  // Start UART tasks for LD2412 communication
  xTaskCreate(uart_event_task, "uart_event_task", 4096, NULL, 5, NULL);
  xTaskCreate(uart_transmission_task, "uart_tx_task", 2048, NULL, 4, NULL);
  xTaskCreate(uart_reception_task, "uart_rx_task", 6144, NULL, 6, NULL);

  ESP_LOGI(TAG, "LD2412 UART tasks started");

  // Wait for UART system to stabilize before sending commands
  delay(1000);

  // Read firmware version first to check module capabilities
  ESP_LOGI(TAG, "Reading LD2412 firmware version");
  read_firmware_version();
  delay(1000);
  ESP_LOGI(TAG, "Enabling LD2412 engineering mode");
  control_engineering_mode();

  // Wait a bit more to ensure the sensor is ready
  delay(1000);

  // Check if system_queue is valid
  if (system_queue == NULL) {
    // ESP_LOGE(TAG, "system_queue is NULL, exiting task");
    vTaskDelete(NULL);
    return;
  }

  // ESP_LOGI(TAG, "system_queue is valid, waiting for sensor data...");

  system_packet sensor_data = {};
  uint32_t no_data_count = 0;
  bool previous_occupancy_state = false; // Track previous occupancy state

  for (;;) {
    // Wait for sensor data from LD2412 UART reception task
    if (xQueueReceive(system_queue, (void *)&sensor_data,
                      pdMS_TO_TICKS(2000))) {
      // Reset no-data counter
      no_data_count = 0;
      // ESP_LOGI(TAG, "Received sensor data packet");

      // Validate data packet
      if (sensor_data.packet_size == 3) {
        uint8_t target_state = sensor_data.data[0];
        int16_t moving_target = sensor_data.data[1];
        int16_t stationary_target = sensor_data.data[2];

        // Suppress unused variable warnings
        (void)moving_target;
        (void)stationary_target;

        // Determine occupancy based on target detection
        bool occupancy_detected = false;

        // Target states: 0 = No target, 1 = Moving target, 2 = Stationary
        // target, 3 = Moving + Stationary
        if (target_state > 0) {
          occupancy_detected = true;
        }

        // Log sensor data
        // Serial.printf("[LD2412] Target State: %d, Moving: %d cm,
        // Stationary:
        // "
        //               "%d cm, Occupancy: %s\r\n",
        //               target_state, moving_target, stationary_target,
        //               occupancy_detected ? "DETECTED" : "CLEAR");

        // Update Zigbee occupancy sensor only if state changed
        if (occupancy_detected != previous_occupancy_state) {
          zbOccupancySensor.setBinaryInput(occupancy_detected);
          zbOccupancySensor.reportBinaryInput(); // Report occupancy status
          previous_occupancy_state =
              occupancy_detected; // Update previous state

          ESP_LOGI(TAG, "Occupancy state changed to: %s",
                   occupancy_detected ? "DETECTED" : "CLEAR");
        }
        // Additional analysis for debugging
        if (occupancy_detected) {
          if (target_state == 1) {
            // ESP_LOGI(TAG, "Moving target detected at %d cm",
            // moving_target);
          } else if (target_state == 2) {
            // ESP_LOGI(TAG, "Stationary target detected at %d cm",
            // stationary_target);
          } else if (target_state == 3) {
            // ESP_LOGI(TAG, "Both targets detected - Moving: %d cm,
            // Stationary: %d cm", moving_target, stationary_target);
          }
        }
      } else {
        // ESP_LOGW(TAG, "Invalid sensor data packet size: %d",
        // sensor_data.packet_size);
      }
    } else {
      // No data received within timeout
      no_data_count++;

      if (no_data_count > 5) {
        // ESP_LOGW(TAG, "No sensor data received for %lu seconds - sensor may
        // be disconnected. Checking UART status...", no_data_count * 2);

        // Additional debugging: Check UART status
        size_t uart_buffer_size;
        uart_get_buffered_data_len(UART_NUM_1, &uart_buffer_size);
        // ESP_LOGW(TAG, "UART buffer has %d bytes", uart_buffer_size);

        // Clear occupancy if no data for extended period
        if (no_data_count > 15) { // 30 seconds of no data
          // Only clear and report if occupancy was previously detected
          if (previous_occupancy_state) {
            ESP_LOGW(TAG, "Extended timeout - clearing occupancy");
            zbOccupancySensor.setBinaryInput(false);
            zbOccupancySensor.reportBinaryInput(); // Report occupancy status
            previous_occupancy_state = false;      // Update previous state
          }

          // Try to recover UART communication
          if (no_data_count > 30) { // 60 seconds, attempt recovery
            // ESP_LOGE(TAG, "Attempting UART recovery after 60 seconds of no
            // data");
            uart_flush_input(UART_NUM_1);
            uart_flush(UART_NUM_1);
            no_data_count = 15; // Reset to prevent immediate re-trigger
          }
        }
      }
    }

    delay(100); // Small delay to prevent excessive CPU usage
  }
}
/******************Speaker Task****************************** */

void speaker_task(void *arg) {
  // Wait for Zigbee network to be ready
  while (!Zigbee.connected()) {
    delay(100);
  }

  ESP_LOGI(TAG, "Speaker task starting...");
  delay(2000); // Allow time for Zigbee to stabilize

  for (;;) {
    // Try to play a WAV file (if it exists)
    ESP_LOGI(TAG, "Attempting to play WAV file: bell.wav");
    speaker.playWavFile("bell.wav");
    ESP_LOGI(TAG, "WAV playback attempt finished");

    ESP_LOGI(TAG, "Waiting 15 seconds before next playback");
    delay(15000); // Wait before next iteration
  }
}

/***************** Main application entry point ****************/

extern "C" void app_main(void) {
  // Initialize Arduino runtime
  initArduino();
  // Setup serial communication
  Serial.begin(115200);
  while (!Serial) {
    ; // Wait for Serial to initialize (for USB CDC targets)
  }
  // Allocate config on heap to avoid stack overflow
  config = (app_config_t *)malloc(sizeof(app_config_t));
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

  // Init button for factory reset
  pinMode(config->factory_reset_pin, INPUT_PULLUP);

  if (config->rgb_led.enabled) {
    pinMode(config->rgb_led.red_pin, OUTPUT);
    pinMode(config->rgb_led.green_pin, OUTPUT);
    pinMode(config->rgb_led.blue_pin, OUTPUT);
    digitalWrite(config->rgb_led.red_pin, LOW);
    digitalWrite(config->rgb_led.green_pin, LOW);
    digitalWrite(config->rgb_led.blue_pin, LOW);
  }

  // set Zigbee device name and model for all endpoints
  zbTempSensor.setManufacturerAndModel(config->device.manufacturer,
                                       config->device.model);
  zbTempHumiditySensor.setManufacturerAndModel(config->device.manufacturer,
                                               config->device.model);
  zbLuxSensor.setManufacturerAndModel(config->device.manufacturer,
                                      config->device.model);
  zbeCo2Sensor.setManufacturerAndModel(config->device.manufacturer,
                                       config->device.model);
  zbTVOCSensor.setManufacturerAndModel(config->device.manufacturer,
                                       config->device.model);
  zbOccupancySensor.setManufacturerAndModel(config->device.manufacturer,
                                            config->device.model);
  zbRangeExtender.setManufacturerAndModel(config->device.manufacturer,
                                          config->device.model);
  zbRgbLight.setManufacturerAndModel(config->device.manufacturer,
                                     config->device.model);
  zbLD2412BluetoothControl.setManufacturerAndModel(config->device.manufacturer,
                                                   config->device.model);

  // Set power source for all endpoints
  if (strcmp(config->device.power_supply, "battery") == 0) {
    zbTempSensor.setPowerSource(ZB_POWER_SOURCE_BATTERY);
    zbTempHumiditySensor.setPowerSource(ZB_POWER_SOURCE_BATTERY);
    zbLuxSensor.setPowerSource(ZB_POWER_SOURCE_BATTERY);
    zbeCo2Sensor.setPowerSource(ZB_POWER_SOURCE_BATTERY);
    zbTVOCSensor.setPowerSource(ZB_POWER_SOURCE_BATTERY);
    zbOccupancySensor.setPowerSource(ZB_POWER_SOURCE_BATTERY);
    zbDBSensor.setPowerSource(ZB_POWER_SOURCE_BATTERY);
    zbRangeExtender.setPowerSource(ZB_POWER_SOURCE_BATTERY);
    zbRgbLight.setPowerSource(ZB_POWER_SOURCE_BATTERY);
    zbLD2412BluetoothControl.setPowerSource(ZB_POWER_SOURCE_BATTERY);
  } else {
    zbTempSensor.setPowerSource(ZB_POWER_SOURCE_MAINS);
    zbTempHumiditySensor.setPowerSource(ZB_POWER_SOURCE_MAINS);
    zbLuxSensor.setPowerSource(ZB_POWER_SOURCE_MAINS);
    zbeCo2Sensor.setPowerSource(ZB_POWER_SOURCE_MAINS);
    zbTVOCSensor.setPowerSource(ZB_POWER_SOURCE_MAINS);
    zbOccupancySensor.setPowerSource(ZB_POWER_SOURCE_MAINS);
    zbDBSensor.setPowerSource(ZB_POWER_SOURCE_MAINS);
    zbRangeExtender.setPowerSource(ZB_POWER_SOURCE_MAINS);
    zbRgbLight.setPowerSource(ZB_POWER_SOURCE_MAINS);
    zbLD2412BluetoothControl.setPowerSource(ZB_POWER_SOURCE_MAINS);
  }

  // Set callback functions for relays change
  zbRelays[0].onLightChange(setRelay0);
  zbRelays[1].onLightChange(setRelay1);
  zbRelays[2].onLightChange(setRelay2);
  zbRelays[3].onLightChange(setRelay3);

  // Set callback function for LD2412 Bluetooth control
  zbLD2412BluetoothControl.onLightChange(onLD2412BluetoothControl);

  // Add OTA client to the temperature sensor endpoint since this is the only
  // one that is always awalable
  zbTempSensor.addOTAClient(OTA_UPGRADE_RUNNING_FILE_VERSION,
                            OTA_UPGRADE_DOWNLOADED_FILE_VERSION,
                            OTA_UPGRADE_HW_VERSION);
  // Add endpoint to Zigbee Core
  if (strcmp(config->device.type, "Router") == 0) {
    Zigbee.addEndpoint(&zbRangeExtender);
  }

  // Add RGB LED endpoint if enabled
  if (config->rgb_led.enabled) {
    zbRgbLight.onLightChange(onRgbLightChange);
    Zigbee.addEndpoint(&zbRgbLight);
    ESP_LOGI(TAG, "RGB LED dimmable light endpoint added");
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
        zbTempHumiditySensor.addHumiditySensor(0, 100,
                                               1); // min = 0%, max = 100%,
                                                   // tolerance = 1%
        Zigbee.addEndpoint(&zbTempHumiditySensor);
      } else if (strcmp(config->sensors[i].type, "ENS160") == 0) {
        // Initialize ENS160 sensor
        Zigbee.addEndpoint(&zbeCo2Sensor);
        Zigbee.addEndpoint(&zbTVOCSensor); // Add TVOC endpoint
      } else if (strcmp(config->sensors[i].type, "INMP441") == 0) {
        // Initialize INMP441 sensor
        // Set up analog input for dB sensor
        zbDBSensor.addAnalogInput();
        zbDBSensor.setAnalogInputDescription("Sound Level (dB)");
        zbDBSensor.setAnalogInputResolution(0.1);
        Zigbee.addEndpoint(&zbDBSensor);
      } else if (strcmp(config->sensors[i].type, "HLK-LD2412") == 0) {
        // Initialize HLK-LD2412 sensor
        // Configure binary sensor for occupancy detection
        zbOccupancySensor.addBinaryInput();
        zbOccupancySensor.setBinaryInputApplication(
            BINARY_INPUT_APPLICATION_TYPE_HVAC_OCCUPANCY);
        zbOccupancySensor.setBinaryInputDescription("Occupancy Sensor");
        Zigbee.addEndpoint(&zbOccupancySensor);
        // Add LD2412 Bluetooth control endpoint (using standard On/Off cluster)
        Zigbee.addEndpoint(&zbLD2412BluetoothControl);
      } else if (strcmp(config->sensors[i].type, "Bluetooth") == 0) {
        // Initialize Bluetooth sensor ToDo für spätere Version
      } else {
        ESP_LOGE(TAG, "Sensor type %s not recognized\n",
                 config->sensors[i].type);
      }
    }
  }
  for (uint8_t i = 0; i < 4; i++) {
    if (config->switches[i].enabled) {
      // Use the initialized array
      ESP_LOGI(TAG, "Configuring binary sensor %d on pin %d", i,
               config->switches[i].pin);

      // Configure binary sensor (more compatible than IAS zones)
      zbBinarySensors[i].setManufacturerAndModel(config->device.manufacturer,
                                                 config->device.model);

      // Set power source for binary sensors
      if (strcmp(config->device.power_supply, "battery") == 0) {
        zbBinarySensors[i].setPowerSource(ZB_POWER_SOURCE_BATTERY);
      } else {
        zbBinarySensors[i].setPowerSource(ZB_POWER_SOURCE_MAINS);
      }

      // Add binary input cluster and set application type
      zbBinarySensors[i].addBinaryInput();
      zbBinarySensors[i].setBinaryInputApplication(
          BINARY_INPUT_APPLICATION_TYPE_SECURITY_INTRUSION_DETECTION);

      zbBinarySensors[i].setBinaryInputDescription(config->switches[i].name);

      // Add binary sensor endpoint (more compatible than IAS zones)
      Zigbee.addEndpoint(&zbBinarySensors[i]);

      pinMode(config->switches[i].pin, INPUT_PULLUP);
      switchNr++;
    }
  }
  ESP_LOGI(TAG, "Total enabled contact switches: %d", switchNr);

  for (uint8_t i = 0; i < 4; i++) {
    if (config->relays[i].enabled) {
      // Initialize relay pin as output and set to OFF
      pinMode(config->relays[i].pin, OUTPUT);
      digitalWrite(config->relays[i].pin, LOW);
      ESP_LOGI(TAG, "Relay %d (%s) initialized on pin %d", i,
               config->relays[i].name, config->relays[i].pin);

      zbRelays[i].setManufacturerAndModel(config->device.manufacturer,
                                          config->device.model);
      if (strcmp(config->device.power_supply, "battery") == 0) {
        zbRelays[i].setPowerSource(ZB_POWER_SOURCE_BATTERY);
      } else {
        zbRelays[i].setPowerSource(ZB_POWER_SOURCE_MAINS);
      }
      Zigbee.addEndpoint(&zbRelays[i]);
    }
  }

  if (config->speaker.enabled) {
    // Initialize speaker endpoint
  }

  // When all EPs are registered, start Zigbee.
  ESP_LOGI(TAG, "Device type: '%s'", config->device.type);

  bool zigbee_started = false;
  if (strcmp(config->device.type, "Router") == 0) {
    ESP_LOGI(TAG, "Starting Zigbee as Router");
    if (!Zigbee.begin(ZIGBEE_ROUTER)) {
      ESP_LOGE(TAG, "Zigbee Router failed to start!");
      Serial.println("Zigbee failed to start!");
      Serial.println("Rebooting...");
      ESP.restart();
    }
    zigbee_started = true;
  } else if (strcmp(config->device.type, "EndDevice") == 0) {
    ESP_LOGI(TAG, "Starting Zigbee as End Device");
    if (!Zigbee.begin(ZIGBEE_END_DEVICE)) {
      ESP_LOGE(TAG, "Zigbee End Device failed to start!");
      Serial.println("Zigbee failed to start!");
      Serial.println("Rebooting...");
      ESP.restart();
    }
    zigbee_started = true;
  } else {
    ESP_LOGE(TAG, "Unknown device type '%s'. Expected 'Router' or 'EndDevice'",
             config->device.type);
    Serial.printf(
        "Unknown device type '%s'. Expected 'Router' or 'EndDevice'\r\n",
        config->device.type);
    Serial.println("Rebooting...");
    ESP.restart();
  }

  if (zigbee_started) {
    ESP_LOGI(TAG,
             "Zigbee started successfully, waiting for network connection...");
    Serial.println("Connecting to network");
    uint32_t connection_timeout = 0;
    while (!Zigbee.connected()) {
      Serial.print(".");
      delay(100);
      connection_timeout++;

      // Add timeout after 5 minutes (3000 * 100ms = 300 seconds)
      if (connection_timeout > 3000) {
        ESP_LOGE(TAG, "Zigbee connection timeout after 5 minutes");
        Serial.println("\nZigbee connection timeout after 5 minutes!");
        Serial.println("Rebooting...");
        ESP.restart();
      }
    }
    Serial.println();
    ESP_LOGI(TAG, "Zigbee network connection established successfully!");

    // Initialize RGB LED state now that Zigbee is connected
    if (config->rgb_led.enabled) {
      zbRgbLight.setLight(true, 100); // Start in ON state
      ESP_LOGI(TAG, "RGB LED dimmable light initialized in on state");
    }
  }

  // Add stabilization delay after network connection
  ESP_LOGI(TAG, "Zigbee network connected. Waiting 10 seconds for network "
                "stabilization...");
  delay(10000);

  // Create sensor tasks only for enabled sensors based on config
  ESP_LOGI(TAG, "Starting sensor tasks with staggered initialization...");

  for (uint8_t i = 0; config->sensors[i].type[0] != '\0'; i++) {
    if (config->sensors[i].enabled) {
      if (strcmp(config->sensors[i].type, "OPT3004") == 0) {
        xTaskCreate(lux_sensor_value_update, "lux_sensor_update", 3072, NULL, 8,
                    NULL);
        zbLuxSensor.setReporting(5, 0,
                                 1000); // min=5s, max=0 (disabled), delta=1000
        ESP_LOGI(TAG, "OPT3004 lux sensor task started");
        delay(2000); // Stagger task creation

      } else if (strcmp(config->sensors[i].type, "MTS4Z") == 0) {
        xTaskCreate(temp_sensor_value_update, "temp_sensor_update", 3072, NULL,
                    8, NULL);
        zbTempSensor.setReporting(5, 0,
                                  5); // min=5s, max=0 (disabled), delta=0.5°C
        ESP_LOGI(TAG, "MTS4Z temperature sensor task started");
        delay(2000); // Stagger task creation

      } else if (strcmp(config->sensors[i].type, "AHT21") == 0) {
        xTaskCreate(temp_humidity_sensor_value_update,
                    "temp_humidity_sensor_update", 4096, NULL, 8, NULL);
        zbTempHumiditySensor.setReporting(
            10, 0, 5); // min=10s, max=0 (disabled), delta=0.5°C
        ESP_LOGI(TAG, "AHT21 temperature/humidity sensor task started");
        delay(2000); // Stagger task creation

      } else if (strcmp(config->sensors[i].type, "ENS160") == 0) {
        zbeCo2Sensor.setReporting(
            30, 0, 50); // min=30s, max=0 (disabled), delta=50 ppm
        zbTVOCSensor.setReporting(
            30, 0, 10); // min=30s, max=0 (disabled), delta=10 ppb
        ESP_LOGI(TAG, "ENS160 air quality sensor reporting configured");
        ESP_LOGI(TAG, "ENS160 TVOC sensor reporting configured");
      } else if (strcmp(config->sensors[i].type, "INMP441") == 0) {
        xTaskCreate(db_sensor_value_update, "db_sensor_update", 8192, NULL, 8,
                    NULL);
        ESP_LOGI(TAG, "INMP441 dB sensor task started");
        delay(2000); // Stagger task creation
      } else if (strcmp(config->sensors[i].type, "HLK-LD2412") == 0) {
        xTaskCreate(occupancy_sensor_value_update, "occupancy_sensor_update",
                    4096, NULL, 7, NULL);
        zbOccupancySensor.setBinaryInput(false); // Initialize occupancy state

        ESP_LOGI(TAG, "HLK-LD2412 occupancy sensor task started");
        delay(2000); // Stagger task creation
      }
    }
  }

  // Wait a bit before starting contact switches
  if (switchNr > 0) {
    ESP_LOGI(TAG, "=== Binary Sensor Setup ===");
    ESP_LOGI(TAG, "Found %d enabled binary sensors (contact switches)",
             switchNr);

    delay(5000);

    ESP_LOGI(TAG, "Starting binary sensor monitoring...");
    xTaskCreate(contact_switches_task, "contact_switches_task", 4096, NULL, 7,
                NULL);
    ESP_LOGI(TAG, "Binary sensor monitoring task started");
  }

  if (config->speaker.enabled) {
    // Initialize speaker endpoint
    ESP_LOGI(TAG, "Speaker endpoint initialized");
    speaker.setPins(config->speaker.bclk, config->speaker.lrc,
                    config->speaker.din);
    // Only setup I2S if we're actually going to use the speaker
    speaker.setupI2S();
    xTaskCreate(speaker_task, "speaker_task", 4096, NULL, 2, NULL);
    ESP_LOGI(TAG, "Speaker task started");
  }

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
    }
    delay(100);
  }

  // Clean up (though this code will never be reached due to infinite loop)
  free(config);
}
