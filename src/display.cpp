#include "globals.h"
#include "display.h"
#include "buzzer.h"
#include <Arduino.h>

void displayClock() {
  DateTime now = rtc.now();
  char timeStr[6];
  sprintf(timeStr, "%02d:%02d", now.hour(), now.minute());

  int startCol = 15 - strlen(timeStr);
  lcd.setCursor(1, 0);
  lcd.print("               ");
  lcd.setCursor(startCol, 0);
  lcd.print(timeStr);
}

void haltAndCatchFire() {
  while (1) {
    playAlertChime();
    delay(1500);
  }
}

void displayScrollingInfo() {
  lcd.setCursor(0, 1);
  switch (displayState) {
  case 0:
    lcd.write((uint8_t)2);
    lcd.print(" Temp: ");
    lcd.print(currentTempF, 1);
    lcd.print("F   ");
    break;
  case 1:
    lcd.write((uint8_t)4);
    lcd.print(" Pres:");
    lcd.print(currentPressureInHg, 2);
    lcd.print("inHg");
    break;
  case 2:
    lcd.write((uint8_t)1);
    lcd.print(" Last: ");
    if (lastDistanceMiles >= 0) {
      lcd.print(lastDistanceMiles);
      lcd.print("mi   ");
    } else {
      lcd.print("None     ");
    }
    break;
  case 3:
    lcd.write((uint8_t)1);
    lcd.print(" Energy: ");
    if (lastEnergy >= 0) {
      lcd.print(lastEnergy);
      lcd.print("      ");
    } else {
      lcd.print("None     ");
    }
    break;
  default:
    displayState = 0;
    return;
  }
  displayState = (displayState + 1) % 4;
}
