#include "Sensoren.h"
#include "RGB.h"
#include <math.h>

static const char *TAG = "Sensoren"; // Logging tag for ESP_LOGI

OPT300x opt3004;

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

/********************* Lux sensor task **************************/
void lux_sensor_value_update(void *arg) {
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

  const double a = 1952.8196544258228;
  const double b = -3.935184782986772;
  const double c = -2020.7691368998812;
  const double n = 40.78635184213555;

  for (;;) {
    OPT300x_S result = opt3004.readResult();
    float lux = result.lux;
    // Serial.printf("[Lux Sensor] Lux: %.2f\r\n", lux);
    if (result.error == NO_ERROR && lux >= 0) {
      // Serial.printf("[Lux Sensor] Lux: %.2f\r\n", lux);
      if (lux > 0) {
        //(kann keine nachkomastellen senden und z2m kann nicht int zu float
        // convertieren, einzige lösung wäre analog cluster verwenden)
        zbLuxSensor.setTemperature(lux);
      } else {
        zbLuxSensor.setTemperature(0.0);
      }
      if (zbRgbLight.getLightState()) {
        // Scale brightness based on lux if lux>1 brightness = lux*0.5 else 0
        float scaledLux = a * pow(lux - b, 1.0 / n) + c;
        uint8_t constrLux = constrain((uint16_t)(scaledLux), 0, 255);
        setRgbLedBrightness(config, lux > 1 ? constrLux : 0);
        // Serial.printf( "[Lux Sensor] Lux: %.2f, Scaled Lux: %.2f, Brightness:
        // %d\r\n", lux,scaledLux, constrLux);
      }

    } else {
      ESP_LOGW(TAG, "Error reading lux sensor or invalid value: %.2f", lux);
    }

    delay(500);
  }
}
