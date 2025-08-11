#ifndef I2S_FULL_DUPLEX_H
#define I2S_FULL_DUPLEX_H

#include "esp_log.h"
#include <driver/i2s_std.h>

class I2SFullDuplex {
public:
  static I2SFullDuplex &getInstance();

  // Initialize the full-duplex I2S with shared pins
  bool initialize(int bclk, int ws, int din, int dout,
                  uint32_t sample_rate_rx = 16000,
                  uint32_t sample_rate_tx = 44100);

  // Get the RX handle for microphone
  i2s_chan_handle_t getRxHandle();

  // Get the TX handle for speaker
  i2s_chan_handle_t getTxHandle();

  // Check if initialized
  bool isInitialized();

  // Cleanup
  void cleanup();

private:
  I2SFullDuplex();
  ~I2SFullDuplex() = default;
  I2SFullDuplex(const I2SFullDuplex &) = delete;
  I2SFullDuplex &operator=(const I2SFullDuplex &) = delete;

  i2s_chan_handle_t _rx_handle = nullptr;
  i2s_chan_handle_t _tx_handle = nullptr;
  bool _initialized = false;
  static const char *TAG;
};

#endif // I2S_FULL_DUPLEX_H
