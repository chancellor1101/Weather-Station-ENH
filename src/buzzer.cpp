#include "globals.h"
#include "buzzer.h"
#include <Arduino.h>

void playStartupChime() {
  tone(BUZZER_PIN, 523, 150);
  delay(200);
  tone(BUZZER_PIN, 659, 150);
  delay(200);
  noTone(BUZZER_PIN);
}

void playAlertChime() {
  tone(BUZZER_PIN, 523, 200);
  delay(250);
  tone(BUZZER_PIN, 440, 200);
  delay(250);
  tone(BUZZER_PIN, 659, 250);
  delay(300);
  noTone(BUZZER_PIN);
}
