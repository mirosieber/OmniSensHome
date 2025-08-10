#include "DbSensor.h"
#include "stdint.h"
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <math.h>
#include <string.h>

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

void DbSensor::begin() {
  i2s_chan_config_t chan_cfg =
      I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_0, I2S_ROLE_MASTER);
  i2s_new_channel(&chan_cfg, nullptr, &_rx_handle);

  i2s_std_config_t std_cfg = {
      .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(_sampleRate),
      .slot_cfg = {.data_bit_width = I2S_DATA_BIT_WIDTH_32BIT,
                   .slot_bit_width = I2S_SLOT_BIT_WIDTH_32BIT,
                   .slot_mode = I2S_SLOT_MODE_MONO,
                   .slot_mask = I2S_STD_SLOT_LEFT,
                   .ws_width = 32,
                   .ws_pol = false,
                   .bit_shift = true,
                   .left_align = false,
                   .big_endian = false,
                   .bit_order_lsb = false},
      .gpio_cfg = {.mclk = I2S_GPIO_UNUSED,
                   .bclk = _bclkPin,
                   .ws = _wsPin,
                   .dout = I2S_GPIO_UNUSED,
                   .din = _dataPin,
                   .invert_flags = {
                       .mclk_inv = false, .bclk_inv = false, .ws_inv = false}}};

  i2s_channel_init_std_mode(_rx_handle, &std_cfg);
  i2s_channel_enable(_rx_handle);
}

float DbSensor::getCurrentDb() {
  int32_t buffer[_bufferSize];
  size_t bytesRead = 0;

  i2s_channel_read(_rx_handle, buffer, sizeof(buffer), &bytesRead,
                   portMAX_DELAY);
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