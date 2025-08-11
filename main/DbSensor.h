#ifndef DB_SENSOR_H
#define DB_SENSOR_H

#pragma once
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <driver/gpio.h>
#include <driver/i2s_std.h>
#include <math.h>
#include <string.h>

class DbSensor {
public:
  DbSensor(gpio_num_t bclk, gpio_num_t ws, gpio_num_t din, uint32_t sampleRate);
  ~DbSensor(); // Add destructor

  void begin();
  void cleanup(); // Add cleanup method
  float getCurrentDb();

  /**
   * @brief Erkennung von Klatschen mit mehreren Prüfungen
   *
   * @param peakThreshold Minimaler Peak-Pegel in dB, um als Kandidat zu gelten.
   * @param dropThreshold Minimaler Abfall in dB nach dropDelayMs, um als
   * Klatschen zu gelten.
   * @param debounceMs Sperrzeit nach einer Erkennung (ms).
   * @param searchMs Dauer des Suchfensters für den Peak (ms).
   * @param dropDelayMs Zeit nach Peak, bis Drop gemessen wird (ms).
   * @param maxImpulseMs Maximale Impulsdauer (ms), längere Signale werden
   * verworfen.
   * @return true Falls ein Klatschen erkannt wurde.
   * @return false Falls kein Klatschen erkannt wurde.
   */
  bool detectClap(float peakThreshold, float dropThresholdMin,
                  float dropThresholdMax, uint32_t debounceMs,
                  uint32_t searchMs, uint32_t dropDelayMs,
                  uint32_t maxImpulseMs);

private:
  gpio_num_t _bclkPin;
  gpio_num_t _wsPin;
  gpio_num_t _dataPin;
  uint32_t _sampleRate;
  i2s_chan_handle_t _rx_handle = nullptr;
  bool _isInitialized = false; // Track initialization state

  static constexpr int _bufferSize = 512; // Anzahl Samples pro Lesevorgang
};

#endif
