#include "DbSensor.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

DbSensor mic(GPIO_NUM_19, GPIO_NUM_18, GPIO_NUM_20, 16000);

extern "C" void app_main2k() {
  mic.begin();

  while (true) {
    if (mic.detectClap(65.0f, // peakThreshold (dB)
                       30.0f, // dropThresholdMin (dB)
                       55.0f, // dropThresholdMax (dB)
                       300,   // debounceMs
                       30,    // searchMs (Peak-Suchfenster)
                       50,    // dropDelayMs (Abfallmessung)
                       50     // maxImpulseMs (max. Impulsdauer)
                       )) {
      // Reaktion auf Klatschen hier
    }
    vTaskDelay(pdMS_TO_TICKS(5));
  }
}
