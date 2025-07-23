#include <Wire.h>
#include <Arduino.h>
void setup() {
  Serial.begin(115200);
  delay(1000);

  Wire.begin(21, 22); // SDA, SCL — adjust if needed
  Serial.println("🔍 I2C Scanner starting...");
}

void loop() {
  byte count = 0;

  for (byte address = 1; address < 127; ++address) {
    Wire.beginTransmission(address);
    if (Wire.endTransmission() == 0) {
      Serial.printf("✅ I2C device found at 0x%02X\n", address);
      count++;
    }
    delay(5);
  }

  if (count == 0)
    Serial.println("❌ No I2C devices found.");
  else
    Serial.printf("🎯 %d device(s) found.\n", count);

  delay(5000); // Wait before rescanning
}
