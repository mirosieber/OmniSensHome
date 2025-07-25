#ifndef DB_SENSOR_H
#define DB_SENSOR_H

#include "driver/gpio.h"
#include "driver/i2s_std.h"
#include <stdint.h>

class DbSensor {
public:
  DbSensor(gpio_num_t bclk, gpio_num_t ws, gpio_num_t din,
           uint32_t sampleRate = 44100);
  void begin();
  float getCurrentDb();

private:
  gpio_num_t _bclkPin, _wsPin, _dataPin;
  uint32_t _sampleRate;
  static const int _bufferSize = 1024;
  i2s_chan_handle_t _rx_handle;
};

#endif
