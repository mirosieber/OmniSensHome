#ifndef SPEAKER_H
#define SPEAKER_H

#include <Arduino.h>
#include <FS.h>
#include <SPIFFS.h>
#include <driver/i2s_std.h>
#include <math.h>
#include <new> // For std::nothrow

#define I2S_SAMPLE_RATE 44100

// WAV file header structure
struct WavHeader {
  char chunkID[4];        // "RIFF"
  uint32_t chunkSize;     // File size - 8
  char format[4];         // "WAVE"
  char subchunk1ID[4];    // "fmt "
  uint32_t subchunk1Size; // 16 for PCM
  uint16_t audioFormat;   // 1 for PCM
  uint16_t numChannels;   // 1 for mono, 2 for stereo
  uint32_t sampleRate;    // Sample rate
  uint32_t byteRate;   // ByteRate = SampleRate * NumChannels * BitsPerSample/8
  uint16_t blockAlign; // BlockAlign = NumChannels * BitsPerSample/8
  uint16_t bitsPerSample; // Bits per sample
  char subchunk2ID[4];    // "data"
  uint32_t subchunk2Size; // Data size
};

class Speaker {
public:
  Speaker(); // Constructor declaration
  void setPins(int bckPin, int wsPin, int dataPin);
  void setupI2S();
  void playTone(int frequency, int duration);
  void playToneAsync(int frequency);
  void playWavFile(const char *filename);
  void playWavFileAsync(const char *filename);
  bool isPlaying();
  void stopTone();
  void stopPlayback();
  void cleanup();

private:
  int _bckPin;
  int _wsPin;
  int _dataPin;
  bool _isInitialized;
  bool _isPlaying;
  i2s_chan_handle_t _tx_handle;
  float _phase;                   // Add phase tracking
  static bool _spiffsInitialized; // Static flag for SPIFFS initialization
  void generateSineWave(int frequency, int16_t *buffer, int samples,
                        int amplitude);
  void resetPhase(); // Add phase reset method
  bool readWavHeader(File &file, WavHeader &header);
  bool readWavHeaderC(FILE *file, WavHeader &header); // C library version
  bool validateWavHeader(const WavHeader &header);
  void convertSampleRate(int16_t *inputBuffer, int inputSamples,
                         uint32_t inputRate, int16_t *outputBuffer,
                         int &outputSamples, uint32_t outputRate);
  bool initializeSPIFFS(); // Helper method for SPIFFS initialization
};

#endif