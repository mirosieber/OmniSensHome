#include "Audio.h"
static const char *TAG = "Audio"; // Logging tag for ESP_LOGI
Speaker speaker;                  // Speaker instance

/************* Audio Trigger Control Functions**************/
void onAudioTriggerControl(app_config_t *config, bool trigger_state) {
  ESP_LOGI(TAG, "Audio trigger Zigbee command received: %s",
           trigger_state ? "ON (TRIGGER)" : "OFF (IGNORE)");
  if (trigger_state) {
    // Only trigger on ON commands (momentary trigger behavior)
    ESP_LOGI(TAG, "Executing audio trigger via Zigbee On/Off cluster");
    triggerAudioPlayback(config);
    ESP_LOGI(TAG, "Audio playback completed");
  } else {
    ESP_LOGI(TAG, "Audio trigger OFF command - no action needed");
  }
}

/************* Audio Trigger Functions **************/
void triggerAudioPlayback(app_config_t *config) {
  if (config->speaker.enabled) {
    ESP_LOGI(TAG, "Audio trigger activated - Playing WAV file");

    // Play audio file immediately when triggered
    speaker.playWavFile("bell.wav");
    ESP_LOGI(TAG, "Audio trigger playback completed");
  }
}
