#include "speaker.h"
#include "esp_spiffs.h"

// Static variable definition
bool Speaker::_spiffsInitialized = false;

Speaker::Speaker() {
  _bckPin = -1;
  _wsPin = -1;
  _dataPin = -1;
  _isInitialized = false;
  _isPlaying = false;
  _tx_handle = NULL;
  _phase = 0.0f;
}

void Speaker::setPins(int bckPin, int wsPin, int dataPin) {
  if (_isInitialized) {
    Serial.println("Warning: I2S already initialized. Call cleanup() first to "
                   "change pins.");
    return;
  }
  _bckPin = bckPin;
  _wsPin = wsPin;
  _dataPin = dataPin;
}

bool Speaker::initializeSPIFFS() {
  if (_spiffsInitialized) {
    return true; // Already initialized
  }

  // SPIFFS is already mounted by configLoader at /spiffs
  // We don't need to call SPIFFS.begin() as it's already mounted via ESP-IDF
  // API
  Serial.println("Using existing SPIFFS mount at /spiffs...");

  // Test if SPIFFS is accessible by trying to open a directory
  // Use standard C library functions since ESP-IDF mounted it
  FILE *test = fopen("/spiffs/config.json", "r");
  if (!test) {
    Serial.println(
        "SPIFFS mount verification failed - config.json not accessible");
    return false;
  }
  fclose(test);

  // Get SPIFFS statistics using ESP-IDF API
  size_t total = 0, used = 0;
  esp_spiffs_info(NULL, &total, &used);

  _spiffsInitialized = true;
  Serial.printf(
      "SPIFFS verified successfully - Total: %d bytes, Used: %d bytes\n", total,
      used);

  // List files for debugging using standard C library
  Serial.println("Checking for WAV files in /spiffs:");

  // Simple check for common audio file extensions
  const char *test_files[] = {"/spiffs/horn.wav", "/spiffs/beep.wav",
                              "/spiffs/sound.wav", "/spiffs/test.wav"};
  for (int i = 0; i < 4; i++) {
    FILE *f = fopen(test_files[i], "r");
    if (f) {
      fseek(f, 0, SEEK_END);
      long size = ftell(f);
      fclose(f);
      Serial.printf("  Found: %s (%ld bytes)\n", test_files[i], size);
    }
  }

  return true;
}

void Speaker::setupI2S() {
  if (_isInitialized) {
    return;
  }

  if (_bckPin == -1 || _wsPin == -1 || _dataPin == -1) {
    Serial.println("Error: I2S pins not set. Call setPins() first.");
    return;
  }

  // Create I2S channel configuration
  i2s_chan_config_t chan_cfg =
      I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_AUTO, I2S_ROLE_MASTER);

  esp_err_t ret = i2s_new_channel(&chan_cfg, &_tx_handle, NULL);
  if (ret != ESP_OK) {
    Serial.printf("Failed to create I2S channel: %d\n", ret);
    return;
  }

  // Configure I2S standard format
  i2s_std_config_t std_cfg = {
      .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(I2S_SAMPLE_RATE),
      .slot_cfg = I2S_STD_MSB_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT,
                                                  I2S_SLOT_MODE_MONO),
      .gpio_cfg =
          {
              .mclk = I2S_GPIO_UNUSED,
              .bclk = static_cast<gpio_num_t>(_bckPin),
              .ws = static_cast<gpio_num_t>(_wsPin),
              .dout = static_cast<gpio_num_t>(_dataPin),
              .din = I2S_GPIO_UNUSED,
              .invert_flags =
                  {
                      .mclk_inv = false,
                      .bclk_inv = false,
                      .ws_inv = false,
                  },
          },
  };

  ret = i2s_channel_init_std_mode(_tx_handle, &std_cfg);
  if (ret == ESP_OK) {
    ret = i2s_channel_enable(_tx_handle);
    if (ret == ESP_OK) {
      _isInitialized = true;
      Serial.println("I2S initialized successfully with new API");
    } else {
      Serial.printf("Failed to enable I2S channel: %d\n", ret);
      i2s_del_channel(_tx_handle);
      _tx_handle = NULL;
    }
  } else {
    Serial.printf("Failed to initialize I2S standard mode: %d\n", ret);
    i2s_del_channel(_tx_handle);
    _tx_handle = NULL;
  }
}

void Speaker::generateSineWave(int frequency, int16_t *buffer, int samples,
                               int amplitude) {
  for (int i = 0; i < samples; ++i) {
    buffer[i] = (int16_t)(amplitude * sin(_phase));
    _phase += (2.0f * PI * frequency) / I2S_SAMPLE_RATE;

    // Keep phase in reasonable range to avoid floating point precision issues
    if (_phase > 2.0f * PI) {
      _phase -= 2.0f * PI;
    }
  }
}

void Speaker::playTone(int frequency, int duration) {
  if (!_isInitialized) {
    setupI2S();
  }

  if (!_isInitialized || _tx_handle == NULL) {
    Serial.println("I2S not initialized, cannot play tone");
    return;
  }

  _isPlaying = true;

  // Reset phase for clean start
  _phase = 0.0f;

  const int amplitude = 5000;
  // Use a larger buffer size for smoother audio (10ms worth of samples)
  const int buffer_size_ms = 10;
  const int samples = (I2S_SAMPLE_RATE * buffer_size_ms) / 1000;
  int16_t *buffer = new int16_t[samples];

  // Calculate total samples needed for the duration
  int total_samples_needed = (I2S_SAMPLE_RATE * duration) / 1000;
  int samples_written = 0;

  while (samples_written < total_samples_needed) {
    int samples_to_write = min(samples, total_samples_needed - samples_written);
    generateSineWave(frequency, buffer, samples_to_write, amplitude);

    size_t bytes_written;
    esp_err_t ret = i2s_channel_write(_tx_handle, buffer,
                                      samples_to_write * sizeof(int16_t),
                                      &bytes_written, pdMS_TO_TICKS(100));
    if (ret != ESP_OK) {
      Serial.printf("I2S write failed: %d\n", ret);
      break;
    }

    samples_written += samples_to_write;
  }

  delete[] buffer;

  _isPlaying = false;

  // Use the improved stopTone method to properly clear DMA buffers
  stopTone();
}

void Speaker::playToneAsync(int frequency) {
  if (!_isInitialized) {
    setupI2S();
  }

  if (!_isInitialized || _tx_handle == NULL) {
    Serial.println("I2S not initialized, cannot play tone");
    return;
  }

  const int amplitude = 5000;
  const int samples = I2S_SAMPLE_RATE / frequency;
  int16_t *buffer = new int16_t[samples];

  generateSineWave(frequency, buffer, samples, amplitude);

  size_t bytes_written;
  esp_err_t ret =
      i2s_channel_write(_tx_handle, buffer, samples * sizeof(int16_t),
                        &bytes_written, portMAX_DELAY);
  if (ret != ESP_OK) {
    Serial.printf("I2S write failed: %d\n", ret);
  }

  delete[] buffer;
}

void Speaker::stopTone() {
  if (_isInitialized && _tx_handle != NULL) {
    // Create a larger silence buffer to clear DMA buffers
    const int samples = 441; // 10ms worth of silence at 44.1kHz
    int16_t *silence = new int16_t[samples];
    memset(silence, 0, samples * sizeof(int16_t));

    // Write silence multiple times to flush DMA buffers
    // Keep the channel enabled to avoid issues with subsequent calls
    for (int i = 0; i < 5; i++) {
      size_t bytes_written;
      esp_err_t ret =
          i2s_channel_write(_tx_handle, silence, samples * sizeof(int16_t),
                            &bytes_written, pdMS_TO_TICKS(50));
      if (ret != ESP_OK) {
        Serial.printf("I2S silence write failed: %d\n", ret);
        break;
      }
    }

    delete[] silence;

    // Reset phase for next tone
    _phase = 0.0f;
  }
}

void Speaker::cleanup() {
  if (_isInitialized && _tx_handle != NULL) {
    _isPlaying = false;
    i2s_channel_disable(_tx_handle);
    i2s_del_channel(_tx_handle);
    _tx_handle = NULL;
    _isInitialized = false;
    Serial.println("I2S driver cleaned up");
  }
}

void Speaker::resetPhase() { _phase = 0.0f; }

bool Speaker::isPlaying() { return _isPlaying; }

void Speaker::stopPlayback() {
  _isPlaying = false;
  stopTone();
}

bool Speaker::readWavHeader(File &file, WavHeader &header) {
  if (file.readBytes((char *)&header, sizeof(WavHeader)) != sizeof(WavHeader)) {
    Serial.println("Failed to read WAV header");
    return false;
  }
  return true;
}

// New function for standard C file operations
bool Speaker::readWavHeaderC(FILE *file, WavHeader &header) {
  // Reset file position to beginning
  fseek(file, 0, SEEK_SET);

  // Read the header in parts to better handle variations
  if (fread(&header.chunkID, 4, 1, file) != 1) {
    Serial.println("Failed to read RIFF header");
    return false;
  }

  if (fread(&header.chunkSize, 4, 1, file) != 1) {
    Serial.println("Failed to read chunk size");
    return false;
  }

  if (fread(&header.format, 4, 1, file) != 1) {
    Serial.println("Failed to read WAVE format");
    return false;
  }

  // Now we need to find the "fmt " chunk (it might not be immediately after
  // WAVE)
  char chunkId[4];
  uint32_t chunkSize;

  while (fread(chunkId, 4, 1, file) == 1) {
    if (fread(&chunkSize, 4, 1, file) != 1) {
      Serial.println("Failed to read chunk size while searching for fmt");
      return false;
    }

    if (strncmp(chunkId, "fmt ", 4) == 0) {
      // Found fmt chunk
      memcpy(header.subchunk1ID, chunkId, 4);
      header.subchunk1Size = chunkSize;

      // Read the fmt chunk data
      if (fread(&header.audioFormat, 2, 1, file) != 1)
        return false;
      if (fread(&header.numChannels, 2, 1, file) != 1)
        return false;
      if (fread(&header.sampleRate, 4, 1, file) != 1)
        return false;
      if (fread(&header.byteRate, 4, 1, file) != 1)
        return false;
      if (fread(&header.blockAlign, 2, 1, file) != 1)
        return false;
      if (fread(&header.bitsPerSample, 2, 1, file) != 1)
        return false;

      // Skip any remaining bytes in the fmt chunk
      if (chunkSize > 16) {
        fseek(file, chunkSize - 16, SEEK_CUR);
      }
      break;
    } else {
      // Skip this chunk
      fseek(file, chunkSize, SEEK_CUR);
    }
  }

  // Now find the "data" chunk
  while (fread(chunkId, 4, 1, file) == 1) {
    if (fread(&chunkSize, 4, 1, file) != 1) {
      Serial.println("Failed to read chunk size while searching for data");
      return false;
    }

    if (strncmp(chunkId, "data", 4) == 0) {
      // Found data chunk
      memcpy(header.subchunk2ID, chunkId, 4);
      header.subchunk2Size = chunkSize;
      // File position is now at the start of audio data
      break;
    } else {
      // Skip this chunk
      fseek(file, chunkSize, SEEK_CUR);
    }
  }

  Serial.printf(
      "WAV Header parsed - Format: %d, Channels: %d, Rate: %lu, Bits: %d\n",
      header.audioFormat, header.numChannels, (unsigned long)header.sampleRate,
      header.bitsPerSample);

  return true;
}

bool Speaker::validateWavHeader(const WavHeader &header) {
  // Check RIFF header
  if (strncmp(header.chunkID, "RIFF", 4) != 0) {
    Serial.printf("Invalid WAV file: missing RIFF header. Found: %.4s\n",
                  header.chunkID);
    return false;
  }

  // Check WAVE format
  if (strncmp(header.format, "WAVE", 4) != 0) {
    Serial.printf("Invalid WAV file: not a WAVE file. Found: %.4s\n",
                  header.format);
    return false;
  }

  // Check fmt chunk
  if (strncmp(header.subchunk1ID, "fmt ", 4) != 0) {
    Serial.printf("Invalid WAV file: missing fmt chunk. Found: %.4s\n",
                  header.subchunk1ID);
    return false;
  }

  // Check data chunk
  if (strncmp(header.subchunk2ID, "data", 4) != 0) {
    Serial.printf("Invalid WAV file: missing data chunk. Found: %.4s\n",
                  header.subchunk2ID);
    return false;
  }

  // Check if it's PCM format
  if (header.audioFormat != 1) {
    Serial.printf("Unsupported audio format: %d (only PCM supported)\n",
                  header.audioFormat);
    return false;
  }

  // Check bits per sample
  if (header.bitsPerSample != 16 && header.bitsPerSample != 8) {
    Serial.printf(
        "Unsupported bits per sample: %d (only 8-bit and 16-bit supported)\n",
        header.bitsPerSample);
    return false;
  }

  // Check if we have valid channels
  if (header.numChannels < 1 || header.numChannels > 2) {
    Serial.printf(
        "Unsupported number of channels: %d (only mono and stereo supported)\n",
        header.numChannels);
    return false;
  }

  // Check sample rate (reasonable range)
  if (header.sampleRate < 8000 || header.sampleRate > 192000) {
    Serial.printf("Unsupported sample rate: %lu Hz\n",
                  (unsigned long)header.sampleRate);
    return false;
  }

  Serial.printf("✓ WAV file validation passed: %lu Hz, %d channels, %d bits\n",
                (unsigned long)header.sampleRate, header.numChannels,
                header.bitsPerSample);
  return true;
}

void Speaker::convertSampleRate(int16_t *inputBuffer, int inputSamples,
                                uint32_t inputRate, int16_t *outputBuffer,
                                int &outputSamples, uint32_t outputRate) {
  if (inputRate == outputRate) {
    // No conversion needed
    memcpy(outputBuffer, inputBuffer, inputSamples * sizeof(int16_t));
    outputSamples = inputSamples;
    return;
  }

  // Simple linear interpolation for sample rate conversion
  float ratio = (float)inputRate / (float)outputRate;
  outputSamples = (int)(inputSamples / ratio);

  for (int i = 0; i < outputSamples; i++) {
    float sourceIndex = i * ratio;
    int index1 = (int)sourceIndex;
    int index2 = index1 + 1;

    if (index2 >= inputSamples) {
      outputBuffer[i] = inputBuffer[inputSamples - 1];
    } else {
      float fraction = sourceIndex - index1;
      outputBuffer[i] = (int16_t)(inputBuffer[index1] * (1.0f - fraction) +
                                  inputBuffer[index2] * fraction);
    }
  }
}

void Speaker::convert8bitTo16bit(uint8_t *input8bit, int16_t *output16bit,
                                 int samples) {
  for (int i = 0; i < samples; i++) {
    // Convert 8-bit unsigned (0-255) to 16-bit signed (-32768 to 32767)
    // 8-bit audio typically uses unsigned values with 128 as the center point
    output16bit[i] = ((int16_t)input8bit[i] - 128) * 256;
  }
}

void Speaker::playWavFile(const char *filename) {
  if (!_isInitialized) {
    setupI2S();
  }

  if (!_isInitialized || _tx_handle == NULL) {
    Serial.println("I2S not initialized, cannot play WAV file");
    return;
  }

  // Initialize SPIFFS if not already done
  if (!initializeSPIFFS()) {
    Serial.println("SPIFFS initialization failed, cannot play WAV file");
    return;
  }

  // Convert filename to use /spiffs mount point
  String fullPath = "/spiffs";
  if (filename[0] != '/') {
    fullPath += "/";
  }
  fullPath += filename;

  Serial.printf("Opening WAV file with C library: %s\n", fullPath.c_str());

  // Use standard C file operations since ESP-IDF SPIFFS mount works
  FILE *wavFile = fopen(fullPath.c_str(), "rb");
  if (!wavFile) {
    Serial.printf("Failed to open WAV file: %s\n", fullPath.c_str());
    return;
  }

  WavHeader header;
  if (!readWavHeaderC(wavFile, header) || !validateWavHeader(header)) {
    fclose(wavFile);
    return;
  }

  _isPlaying = true;
  Serial.printf("Playing WAV file: %s\n", filename);
  Serial.printf("WAV Format: %lu Hz, %d channels, %d bits\n",
                (unsigned long)header.sampleRate, header.numChannels,
                header.bitsPerSample);

  // Use smaller, safer buffer sizes and more careful memory management
  const size_t maxBufferSize = 512; // Smaller buffer to reduce memory pressure
  int16_t *readBuffer = nullptr;
  int16_t *outputBuffer = nullptr;
  uint8_t *raw8bitBuffer = nullptr; // For 8-bit audio data

  // Calculate proper buffer sizes based on audio format
  size_t readBufferSize = maxBufferSize;
  if (header.numChannels == 2) {
    readBufferSize = maxBufferSize * 2; // Need space for stereo samples
  }

  // For 8-bit audio, we need a separate buffer for raw data
  if (header.bitsPerSample == 8) {
    size_t raw8bitBufferSize = readBufferSize;
    raw8bitBuffer = new (std::nothrow) uint8_t[raw8bitBufferSize];
    if (!raw8bitBuffer) {
      Serial.println("Failed to allocate memory for 8-bit audio buffer");
      fclose(wavFile);
      _isPlaying = false;
      return;
    }
  }

  // Calculate maximum output buffer size for sample rate conversion
  size_t outputBufferSize = maxBufferSize;
  if (header.sampleRate != 44100) {
    float ratio = 44100.0f / (float)header.sampleRate;
    outputBufferSize =
        (size_t)(maxBufferSize * ratio) + 64; // Add safety margin
  }

  // Allocate buffers with proper error checking
  readBuffer = new (std::nothrow) int16_t[readBufferSize];
  outputBuffer = new (std::nothrow) int16_t[outputBufferSize];

  if (!readBuffer || !outputBuffer) {
    Serial.println("Failed to allocate memory for WAV playback buffers");
    delete[] readBuffer;
    delete[] outputBuffer;
    delete[] raw8bitBuffer;
    fclose(wavFile);
    _isPlaying = false;
    return;
  }

  uint32_t totalSamples = header.subchunk2Size / (header.bitsPerSample / 8);
  uint32_t samplesProcessed = 0;

  while (_isPlaying && samplesProcessed < totalSamples) {
    // Calculate how many samples to read this iteration
    size_t samplesToRead = maxBufferSize;
    if (header.numChannels == 2) {
      samplesToRead = maxBufferSize; // Will be converted to mono later
    }

    // Don't read beyond end of file
    if (samplesToRead > (totalSamples - samplesProcessed)) {
      samplesToRead = totalSamples - samplesProcessed;
    }

    size_t samplesRead;

    // Handle different bit depths
    if (header.bitsPerSample == 8) {
      // Read 8-bit data and convert to 16-bit
      samplesRead =
          fread(raw8bitBuffer, sizeof(uint8_t), samplesToRead, wavFile);
      if (samplesRead == 0) {
        break; // End of file or error
      }
      // Convert 8-bit to 16-bit
      convert8bitTo16bit(raw8bitBuffer, readBuffer, samplesRead);
    } else {
      // Read 16-bit data directly
      samplesRead = fread(readBuffer, sizeof(int16_t), samplesToRead, wavFile);
      if (samplesRead == 0) {
        break; // End of file or error
      }
    }

    // Convert channels if necessary (stereo to mono)
    size_t monoSamples = samplesRead;
    if (header.numChannels == 2) {
      // Convert stereo to mono by averaging both channels
      for (size_t i = 0; i < samplesRead / 2; i++) {
        int32_t avg =
            ((int32_t)readBuffer[i * 2] + (int32_t)readBuffer[i * 2 + 1]) / 2;
        readBuffer[i] = (int16_t)avg;
      }
      monoSamples = samplesRead / 2;
    }

    // Convert sample rate if necessary
    int outputSamples = monoSamples;
    int16_t *finalBuffer = readBuffer;

    if (header.sampleRate != 44100) {
      convertSampleRate(readBuffer, monoSamples, header.sampleRate,
                        outputBuffer, outputSamples, 44100);
      finalBuffer = outputBuffer;
    }

    // Write to I2S with bounds checking
    if (outputSamples > 0 && outputSamples <= (int)outputBufferSize) {
      size_t bytes_written;
      esp_err_t ret = i2s_channel_write(_tx_handle, finalBuffer,
                                        outputSamples * sizeof(int16_t),
                                        &bytes_written, portMAX_DELAY);

      if (ret != ESP_OK) {
        Serial.printf("I2S write error: %s\n", esp_err_to_name(ret));
        break;
      }
    } else {
      Serial.printf("Invalid output samples count: %d\n", outputSamples);
      break;
    }

    samplesProcessed += samplesRead;

    // Small delay to prevent watchdog timeout and allow other tasks to run
    if (samplesProcessed % 4096 == 0) {
      delay(1);
    }
  }

  _isPlaying = false;
  fclose(wavFile);

  // Clean up memory
  delete[] readBuffer;
  delete[] outputBuffer;
  delete[] raw8bitBuffer; // Clean up 8-bit buffer if allocated

  // Clear I2S buffers with silence to ensure complete silence after playback
  // This matches what we do in stopTone() method
  if (_tx_handle != NULL) {
    const int silenceSamples = 441; // 10ms worth of silence at 44.1kHz
    int16_t *silence = new (std::nothrow) int16_t[silenceSamples];
    if (silence) {
      memset(silence, 0, silenceSamples * sizeof(int16_t));

      // Write silence multiple times to flush DMA buffers
      for (int i = 0; i < 5; i++) {
        size_t bytes_written;
        esp_err_t ret = i2s_channel_write(_tx_handle, silence,
                                          silenceSamples * sizeof(int16_t),
                                          &bytes_written, pdMS_TO_TICKS(50));
        if (ret != ESP_OK) {
          Serial.printf("I2S silence write failed: %s\n", esp_err_to_name(ret));
          break;
        }
      }
      delete[] silence;
    }
  }

  Serial.printf("WAV playback finished. Processed %lu samples\n",
                (unsigned long)samplesProcessed);
}

void Speaker::playWavFileAsync(const char *filename) {
  // Initialize SPIFFS if not already done
  if (!initializeSPIFFS()) {
    return;
  }

  // For async playback, you could implement this using a FreeRTOS task
  // For now, just call the synchronous version
  playWavFile(filename);
}
