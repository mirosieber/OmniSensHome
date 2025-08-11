#include "DbSensor.h"
#include "I2SFullDuplex.h"
#include "esp_log.h"
#include "stdint.h"
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <math.h>
#include <string.h>

static const char *TAG = "DbSensor";

// Einfacher 1. Ordnung Hochpass (Butterworth-artig)
class SimpleHighpass {
public:
  SimpleHighpass(float sampleRate, float cutoffHz) {
    float RC = 1.0f / (2.0f * M_PI * cutoffHz);
    alpha = RC / (RC + 1.0f / sampleRate);
    y_last = 0.0f;
    x_last = 0.0f;
  }
  float process(float x) {
    float y = alpha * (y_last + x - x_last);
    y_last = y;
    x_last = x;
    return y;
  }

private:
  float alpha;
  float y_last;
  float x_last;
};

// Hochpassinstanz für 1 kHz
static SimpleHighpass hp(16000.0f, 1000.0f);

DbSensor::DbSensor(gpio_num_t bclk, gpio_num_t ws, gpio_num_t din,
                   uint32_t sampleRate)
    : _bclkPin(bclk), _wsPin(ws), _dataPin(din), _sampleRate(sampleRate) {}

DbSensor::~DbSensor() { cleanup(); }

void DbSensor::begin() {
  ESP_LOGI(TAG, "Initializing DbSensor using shared I2S instance");

  // Use shared full-duplex I2S instance
  I2SFullDuplex &i2s = I2SFullDuplex::getInstance();

  // Initialize shared I2S if not already done
  if (!i2s.isInitialized()) {
    ESP_LOGI(TAG, "Initializing shared I2S for microphone and speaker");
    bool success = i2s.initialize(_bclkPin,    // BCLK (19)
                                  _wsPin,      // WS (18)
                                  _dataPin,    // DIN (20 - microphone)
                                  GPIO_NUM_3,  // DOUT (3 - speaker)
                                  _sampleRate, // RX sample rate (16000)
                                  44100        // TX sample rate (44100)
    );
    if (!success) {
      ESP_LOGE(TAG, "Failed to initialize shared I2S");
      return;
    }
  }

  // Get RX handle for microphone
  _rx_handle = i2s.getRxHandle();
  if (!_rx_handle) {
    ESP_LOGE(TAG, "Failed to get I2S RX handle");
    return;
  }

  _isInitialized = true;
  ESP_LOGI(TAG, "DbSensor initialized successfully using shared I2S instance");
}

void DbSensor::cleanup() {
  if (_isInitialized) {
    ESP_LOGI(TAG, "Cleaning up DbSensor (shared I2S remains active)");
    _rx_handle = nullptr; // Don't delete shared handle
    _isInitialized = false;
    ESP_LOGI(TAG, "DbSensor cleaned up");
  }
}

float DbSensor::getCurrentDb() {
  if (!_isInitialized || _rx_handle == nullptr) {
    ESP_LOGW(TAG, "DbSensor not initialized, returning minimum dB value");
    return -120.0f;
  }

  int32_t buffer[_bufferSize];
  size_t bytesRead = 0;

  esp_err_t ret = i2s_channel_read(_rx_handle, buffer, sizeof(buffer),
                                   &bytesRead, portMAX_DELAY);
  if (ret != ESP_OK) {
    ESP_LOGW(TAG, "I2S read failed: %s", esp_err_to_name(ret));
    return -120.0f;
  }

  int samples = bytesRead / sizeof(int32_t);
  if (samples == 0)
    return -120.0f;

  int64_t sum = 0;
  for (int i = 0; i < samples; ++i) {
    int32_t raw = buffer[i] >> 14; // 18-bit aus 32-bit Container
    float filtered = hp.process((float)raw);
    sum += (int64_t)(filtered * filtered);
  }

  float rms = sqrtf((float)sum / samples);
  if (rms < 1e-9)
    return -120.0f;

  return 20.0f * log10f(rms);
}

bool DbSensor::detectClap(float peakThreshold, float dropThresholdMin,
                          float dropThresholdMax, uint32_t debounceMs,
                          uint32_t searchMs, uint32_t dropDelayMs,
                          uint32_t maxImpulseMs) {
  static uint32_t lastClapTime = 0;
  uint32_t now = xTaskGetTickCount() * portTICK_PERIOD_MS;

  if (now - lastClapTime < debounceMs)
    return false;

  // 1️⃣ Peak in Suchfenster bestimmen
  float peakDb = -120.0f;
  for (uint32_t t = 0; t < searchMs; t += 2) {
    float db = getCurrentDb();
    if (db > peakDb)
      peakDb = db;
    vTaskDelay(pdMS_TO_TICKS(2));
  }

  if (peakDb < peakThreshold)
    return false;

  // 2️⃣ Nach Peak warten und Drop messen
  float sumAfter = 0;
  int count = 0;
  for (uint32_t t = 0; t < dropDelayMs; t += 10) {
    vTaskDelay(pdMS_TO_TICKS(10));
    sumAfter += getCurrentDb();
    count++;
  }
  float afterDb = sumAfter / count;
  float drop = peakDb - afterDb;

  printf("[DEBUG] Peak: %.2f dB | After: %.2f dB | Drop: %.2f dB\n", peakDb,
         afterDb, drop);

  if (drop < dropThresholdMin || drop > dropThresholdMax) {
    return false;
  }

  // 3️⃣ Impulsdauer messen
  uint32_t start = xTaskGetTickCount() * portTICK_PERIOD_MS;
  while ((xTaskGetTickCount() * portTICK_PERIOD_MS - start) < maxImpulseMs) {
    if (getCurrentDb() < (peakDb - dropThresholdMin))
      break;
    vTaskDelay(pdMS_TO_TICKS(5));
  }
  uint32_t duration = (xTaskGetTickCount() * portTICK_PERIOD_MS) - start;

  if (duration > maxImpulseMs) {
    printf("[DEBUG] ❌ Dauer zu lang (%lu ms)\n", duration);
    return false;
  }

  // ✅ Klatschen erkannt
  lastClapTime = now;
  printf("🎉 Klatschen erkannt! Dauer: %lu ms\n", duration);
  return true;
}
