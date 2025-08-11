#include "I2SFullDuplex.h"

const char *I2SFullDuplex::TAG = "I2SFullDuplex";

I2SFullDuplex &I2SFullDuplex::getInstance() {
  static I2SFullDuplex instance;
  return instance;
}

I2SFullDuplex::I2SFullDuplex() {}

bool I2SFullDuplex::initialize(int bclk, int ws, int din, int dout,
                               uint32_t sample_rate_rx,
                               uint32_t sample_rate_tx) {
  if (_initialized) {
    ESP_LOGW(TAG, "I2S already initialized");
    return true;
  }

  ESP_LOGI(
      TAG,
      "Initializing full-duplex I2S on pins BCLK:%d, WS:%d, DIN:%d, DOUT:%d",
      bclk, ws, din, dout);

  // Create I2S channel pair for full-duplex operation
  i2s_chan_config_t chan_cfg =
      I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_0, I2S_ROLE_MASTER);

  esp_err_t ret = i2s_new_channel(&chan_cfg, &_tx_handle, &_rx_handle);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to create I2S channel pair: %s",
             esp_err_to_name(ret));
    return false;
  }

  // Configure RX channel (microphone)
  i2s_std_config_t rx_cfg = {
      .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(sample_rate_rx),
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
                   .bclk = static_cast<gpio_num_t>(bclk),
                   .ws = static_cast<gpio_num_t>(ws),
                   .dout = I2S_GPIO_UNUSED,
                   .din = static_cast<gpio_num_t>(din),
                   .invert_flags = {
                       .mclk_inv = false, .bclk_inv = false, .ws_inv = false}}};

  ret = i2s_channel_init_std_mode(_rx_handle, &rx_cfg);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to initialize RX channel: %s", esp_err_to_name(ret));
    i2s_del_channel(_tx_handle);
    i2s_del_channel(_rx_handle);
    _tx_handle = nullptr;
    _rx_handle = nullptr;
    return false;
  }

  // Configure TX channel (speaker)
  i2s_std_config_t tx_cfg = {
      .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(sample_rate_tx),
      .slot_cfg = I2S_STD_MSB_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT,
                                                  I2S_SLOT_MODE_MONO),
      .gpio_cfg = {.mclk = I2S_GPIO_UNUSED,
                   .bclk = static_cast<gpio_num_t>(bclk), // Shared BCLK
                   .ws = static_cast<gpio_num_t>(ws),     // Shared WS
                   .dout = static_cast<gpio_num_t>(dout),
                   .din = I2S_GPIO_UNUSED,
                   .invert_flags = {
                       .mclk_inv = false, .bclk_inv = false, .ws_inv = false}}};

  ret = i2s_channel_init_std_mode(_tx_handle, &tx_cfg);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to initialize TX channel: %s", esp_err_to_name(ret));
    i2s_del_channel(_tx_handle);
    i2s_del_channel(_rx_handle);
    _tx_handle = nullptr;
    _rx_handle = nullptr;
    return false;
  }

  // Enable both channels
  ret = i2s_channel_enable(_rx_handle);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to enable RX channel: %s", esp_err_to_name(ret));
    cleanup();
    return false;
  }

  ret = i2s_channel_enable(_tx_handle);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to enable TX channel: %s", esp_err_to_name(ret));
    cleanup();
    return false;
  }

  _initialized = true;
  ESP_LOGI(TAG, "Full-duplex I2S initialized successfully");
  return true;
}

i2s_chan_handle_t I2SFullDuplex::getRxHandle() { return _rx_handle; }

i2s_chan_handle_t I2SFullDuplex::getTxHandle() { return _tx_handle; }

bool I2SFullDuplex::isInitialized() { return _initialized; }

void I2SFullDuplex::cleanup() {
  if (_initialized) {
    if (_rx_handle) {
      i2s_channel_disable(_rx_handle);
      i2s_del_channel(_rx_handle);
      _rx_handle = nullptr;
    }
    if (_tx_handle) {
      i2s_channel_disable(_tx_handle);
      i2s_del_channel(_tx_handle);
      _tx_handle = nullptr;
    }
    _initialized = false;
    ESP_LOGI(TAG, "Full-duplex I2S cleaned up");
  }
}
