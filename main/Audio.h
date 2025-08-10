#ifndef AUDIO_H
#define AUDIO_H

#include "Arduino.h"
#include "configLoader.h"
#include "speaker.h"

extern Speaker speaker; // Speaker instance

void onAudioTriggerControl(app_config_t *config, bool trigger_state);
void triggerAudioPlayback(app_config_t *config);
void speaker_task(void *arg);

#endif // AUDIO_H