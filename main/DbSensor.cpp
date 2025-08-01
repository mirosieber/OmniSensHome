#include "DbSensor.h"
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <math.h>

DbSensor::DbSensor(gpio_num_t bclk, gpio_num_t ws, gpio_num_t din,
                   uint32_t sampleRate)
    : _bclkPin(bclk), _wsPin(ws), _dataPin(din), _sampleRate(sampleRate) {}

void DbSensor::begin() {
  i2s_chan_config_t chan_cfg =
      I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_0, I2S_ROLE_MASTER);
  i2s_new_channel(&chan_cfg, nullptr, &_rx_handle);

  i2s_std_config_t std_cfg = {
      .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(_sampleRate),
      .slot_cfg = {.data_bit_width =
                       I2S_DATA_BIT_WIDTH_32BIT, // Wichtig: 32-bit Container
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

  int64_t sum = 0;
  for (int i = 0; i < samples; ++i) {
    // Exakt wie im funktionierenden Beispiel:
    // 1. Rohdaten verwenden (32-bit Container)
    // 2. 14 Bits nach rechts schieben
    int32_t sample = buffer[i] >> 14;
    sum += static_cast<int64_t>(sample) * sample;
  }

  if (samples == 0)
    return -120.0f;

  float rms = sqrtf(static_cast<float>(sum) / samples);

  // dB-Berechnung mit Untergrenze
  if (rms < 1e-9)
    return -120.0f;

  float dB = 20.0 * log10(rms);
  return dB;
}