
#include "Arduino.h"
#include "Pin.h"
#include "esp_log.h"
#include "esp_zigbee_core.h"
#include "esp_zigbee_ota.h"
#include <AHTxx.h>
#include <MTS4x.h>
#include <OPT300x.h>
#include <ScioSense_ENS16x.h>
#include <Wire.h>

extern "C" void app_main() {
  // Initialize Arduino runtime
  initArduino();

  // Setup serial communication
  Serial.begin(115200);
  while (!Serial) {
    ; // Wait for Serial to initialize (for USB CDC targets like ESP32-S2/S3)
  }

  Serial.println("ESP32 Arduino core initialized inside app_main().");

  // Simple loop: print a message every 2 seconds
  int counter = 0;
  while (true) {
    Serial.print("Hello from ESP32! Counter: ");
    Serial.println(counter++);
    delay(2000); // Wait for 2 seconds
  }
}
