
#include "Arduino.h"
#include "Pin.h"
#include "Zigbee.h"
#include "esp_log.h"
#include "esp_zigbee_core.h"
#include "esp_zigbee_ota.h"
// #include <AHTxx.h>
// #include <MTS4x.h>
// #include <OPT300x.h>
// #include <ScioSense_ENS16x.h>
// #include <Wire.h>

/* Zigbee light bulb configuration */
#define ZIGBEE_LIGHT_ENDPOINT 1
uint8_t led = RGB_BLAU;
uint8_t button = BOOT_PIN;

/* Zigbee OTA configuration */
#define OTA_UPGRADE_RUNNING_FILE_VERSION                                       \
  0x01010100 // Increment this value when the running image is updated
#define OTA_UPGRADE_DOWNLOADED_FILE_VERSION                                    \
  0x01010101 // Increment this value when the downloaded image is updated
#define OTA_UPGRADE_HW_VERSION                                                 \
  0x0101 // The hardware version, this can be used to differentiate between
         // different hardware versions

ZigbeeLight zbLight = ZigbeeLight(ZIGBEE_LIGHT_ENDPOINT);

/********************* RGB LED functions **************************/
void setLED(bool value) { digitalWrite(led, value); }

extern "C" void app_main() {
  // Initialize Arduino runtime
  initArduino();

  // Setup serial communication
  Serial.begin(115200);
  while (!Serial) {
    ; // Wait for Serial to initialize (for USB CDC targets like ESP32-S2/S3)
  }

  Serial.println("ESP32 Arduino core initialized inside app_main().");

  // setup
  //  Init LED and turn it OFF (if LED_PIN == RGB_BUILTIN, the rgbLedWrite()
  //  will be used under the hood)
  pinMode(led, OUTPUT);
  digitalWrite(led, LOW);

  // Init button for factory reset
  pinMode(button, INPUT_PULLUP);

  // Optional: set Zigbee device name and model
  zbLight.setManufacturerAndModel("Espressif", "ZBLightBulb");

  // Set callback function for light change
  zbLight.onLightChange(setLED);

  // Add OTA client to the light bulb
  zbLight.addOTAClient(OTA_UPGRADE_RUNNING_FILE_VERSION,
                       OTA_UPGRADE_DOWNLOADED_FILE_VERSION,
                       OTA_UPGRADE_HW_VERSION);

  // Add endpoint to Zigbee Core
  Serial.println("Adding ZigbeeLight endpoint to Zigbee Core");
  Zigbee.addEndpoint(&zbLight);

  // When all EPs are registered, start Zigbee. By default acts as
  // ZIGBEE_END_DEVICE
  if (!Zigbee.begin()) {
    Serial.println("Zigbee failed to start!");
    Serial.println("Rebooting...");
    ESP.restart();
  }
  Serial.println("Connecting to network");
  while (!Zigbee.connected()) {
    Serial.print(".");
    delay(100);
  }
  Serial.println();

  // Start Zigbee OTA client query, first request is within a minute and the
  // next requests are sent every hour automatically
  zbLight.requestOTAUpdate();

  // Simple loop: print a message every 2 seconds
  int counter = 0;
  while (true) {
    // Checking button for factory reset
    if (digitalRead(button) == LOW) { // Push button pressed
      // Key debounce handling
      delay(100);
      int startTime = millis();
      while (digitalRead(button) == LOW) {
        delay(50);
        if ((millis() - startTime) > 3000) {
          // If key pressed for more than 3secs, factory reset Zigbee and reboot
          Serial.println("Resetting Zigbee to factory and rebooting in 1s.");
          delay(1000);
          Zigbee.factoryReset();
        }
      }
      // Toggle light by pressing the button
      zbLight.setLight(!zbLight.getLightState());
    }
    delay(100);
  }
}
