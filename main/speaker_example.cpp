/*
 * Example usage of the Speaker class with I2S
 *
 * This example shows how to use the Speaker class to play tones
 * using I2S audio output.
 */

#include "speaker.h"

// Pin definitions for I2S
#define I2S_BCK_IO 26 // Bit Clock
#define I2S_WS_IO 25  // Word Select (LRCLK)
#define I2S_DO_IO 22  // Data Out

// Create speaker instance
Speaker speaker(I2S_BCK_IO, I2S_WS_IO, I2S_DO_IO);

void setup() {
  Serial.begin(115200);
  Serial.println("Starting I2S Speaker Example");

  // Initialize the speaker
  speaker.setupI2S();

  // Play a 440Hz tone (A4 note) for 1 second
  Serial.println("Playing 440Hz tone for 1 second");
  speaker.playTone(440, 1000);

  delay(500);

  // Play a 880Hz tone (A5 note) for 500ms
  Serial.println("Playing 880Hz tone for 500ms");
  speaker.playTone(880, 500);
}

void loop() {
  // Play some musical notes in sequence
  int notes[] = {261, 294, 329, 349, 392, 440, 493, 523}; // C major scale
  int noteDuration = 300;

  for (int i = 0; i < 8; i++) {
    Serial.printf("Playing note: %d Hz\n", notes[i]);
    speaker.playTone(notes[i], noteDuration);
    delay(100); // Small pause between notes
  }

  delay(2000); // Wait 2 seconds before repeating
}
