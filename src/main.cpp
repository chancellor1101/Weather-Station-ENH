#include <SPI.h>
#include <Wire.h>
#include "SparkFun_AS3935.h"
#include "RTClib.h"
#include <LiquidCrystal_I2C.h>
#include <Adafruit_BMP280.h>

#define INDOOR 0x12
#define OUTDOOR 0xE
#define LIGHTNING_INT 0x08
#define DISTURBER_INT 0x04
#define NOISE_INT 0x01
#define BUZZER_PIN 13

LiquidCrystal_I2C lcd(0x27, 16, 2); // Address, 16 chars, 2 lines
SparkFun_AS3935 lightning;
RTC_DS3231 rtc;
Adafruit_BMP280 bmp; // I2C mode

const int lightningInt = 26;
int spiCS = 27;

int intVal = 0;
int noise = 2;
// int disturber = 2; // This variable is no longer needed if setDisturberDetection is removed

unsigned long lastAlertMillis = 0;
const unsigned long alertCooldown = 10UL * 60UL * 1000UL; // 10 minutes
unsigned long lastCycle = 0;
int displayState = 0;
bool alertActive = false;
unsigned long alertStartMillis = 0;
int lastClockMinute = -1; // for tracking minute change

// --- NEW/MODIFIED GLOBAL VARIABLES ---
float currentTempF = 0.0;     // Temperature in Fahrenheit
float currentPressureInHg = 0.0; // Pressure in inches of Hg
int lastDistanceMiles = -1;   // Distance in miles
int lastEnergy = -1;          // ADDED: lastEnergy declaration

unsigned long lastLightningTimeMillis = 0; // Timestamp of the last lightning event
const unsigned long lightningClearCooldown = 1UL * 60UL * 60UL * 1000UL; // 1 hour in milliseconds

// --- CUSTOM CHARACTER ARRAYS ---
byte signalBarsFull[8] = {
  0b00000, // Row 0
  0b00001, // Row 1: . . . . X
  0b00011, // Row 2: . . . X X
  0b00111, // Row 3: . . X X X
  0b01111, // Row 4: . X X X X
  0b11111, // Row 5: X X X X X
  0b00000, // Row 6
  0b00000  // Row 7
};

byte lightningBoltDiagonal[8] = {
  0b10000, // X . . . .
  0b11000, // X X . . .
  0b01000, // . X . . .
  0b00100, // . . X . .
  0b00010, // . . . X .
  0b00110, // . . X X .
  0b01000, // . X . . .
  0b10000  // X . . . .
};

byte thermometer[8] = {B00100, B01010, B01010, B01110, B01110, B01110, B11111, B01110};
byte droplet[8] = {B00100, B00100, B01010, B01010, B10001, B10001, B10001, B01110};
byte pressure[8] = {B00000, B01110, B10001, B10101, B10001, B01110, B00000, B00000};


// --- BUZZER FUNCTIONS ---
void playStartupChime()
{
  tone(BUZZER_PIN, 523, 150);
  delay(200);
  tone(BUZZER_PIN, 659, 150);
  delay(200);
  noTone(BUZZER_PIN);
}

void playAlertChime()
{
  tone(BUZZER_PIN, 523, 200);
  delay(250);
  tone(BUZZER_PIN, 440, 200);
  delay(250);
  tone(BUZZER_PIN, 659, 250);
  delay(300);
  noTone(BUZZER_PIN);
}


// --- DISPLAY FUNCTIONS ---
void displayClock()
{
  DateTime now = rtc.now();
  char timeStr[6];
  sprintf(timeStr, "%02d:%02d", now.hour(), now.minute()); // Still using 24-hour for compactness

  int startCol = 15 - strlen(timeStr); // right-align
  lcd.setCursor(1, 0); // Start clearing from a safe spot, not column 0 where WiFi icon is
  lcd.print("               "); // Clear the rest of the line
  lcd.setCursor(startCol, 0);
  lcd.print(timeStr);
}


void haltAndCatchFire()
{
  while (1)
  {
    playAlertChime();
    delay(1500); // Wait for 1.5 seconds
  };
}

void displayScrollingInfo()
{
  lcd.setCursor(0, 1);
  switch (displayState)
  {
  case 0:
    lcd.write((uint8_t)2); // thermometer
    lcd.print(" Temp: ");
    lcd.print(currentTempF, 1); // Display Fahrenheit
    lcd.print("F   ");
    break;
  case 1:
    lcd.write((uint8_t)4); // pressure (using the new pressure icon)
    lcd.print(" Pres:");
    lcd.print(currentPressureInHg, 2); // Display inHg with 2 decimal places
    lcd.print("inHg");
    break;
  case 2:
    lcd.write((uint8_t)1); // lightning bolt
    lcd.print(" Last: ");
    if (lastDistanceMiles >= 0) // Check new distance variable
    {
      lcd.print(lastDistanceMiles);
      lcd.print("mi   "); // Change unit to miles
    }
    else
    {
      lcd.print("None     ");
    }
    break;
  case 3:
    lcd.write((uint8_t)1); // lightning bolt
    lcd.print(" Energy: ");
    if (lastEnergy >= 0) // lastEnergy is now declared
    {
      lcd.print(lastEnergy);
      lcd.print("      ");
    }
    else
    {
      lcd.print("None     ");
    }
    break;
  default:
    displayState = 0;
    return;
  }
  displayState = (displayState + 1) % 4;
}

void setup()
{
  pinMode(lightningInt, INPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  Serial.begin(115200);
  SPI.begin();
  lcd.init();
  lcd.backlight();

  // Create custom characters
  lcd.createChar(0, signalBarsFull);
  lcd.createChar(1, lightningBoltDiagonal);
  lcd.createChar(2, thermometer);
  lcd.createChar(3, droplet); // Droplet might be good for humidity, but not used now
  lcd.createChar(4, pressure); // New pressure icon!
  lcd.clear();

  lcd.setCursor(0, 0);
  lcd.write((uint8_t)0); // Display the Wi-Fi icon during startup message
  lcd.print(" System Start");
  delay(2000); // Wait for sensors and LCD to warm up

  if (!lightning.beginSPI(spiCS))
  {
    lcd.setCursor(0, 1);
    lcd.print("Sensor Failure");
    haltAndCatchFire();
  }
  lightning.setNoiseLevel(noise);
  lightning.setIndoorOutdoor(OUTDOOR); // Assuming outdoor usage
  // Removed: lightning.setDisturberDetection(disturber); // Removed due to error

  if (!rtc.begin()) // This function already checks if the RTC is running and ready
  {
    lcd.setCursor(0, 1);
    lcd.print("Clock Failure ");
    haltAndCatchFire();
  }
  // If RTC is not running at all (e.g., first boot or dead battery),
  // rtc.begin() would return false and halt. If it returns true but time is lost,
  // you might need to manually set it or implement a more sophisticated check.
  // For most cases, if rtc.begin() is true, it's either running or just started.
  // This line sets the RTC to the date & time this sketch was compiled if the RTC loses time
  // but rtc.begin() still reports success (less common scenario for a truly dead RTC)
  // or if rtc.adjust() is needed to sync an already 'running' but wrong clock.
  // A robust check for time validity might involve checking the year (e.g., if (now.year() < 2023)).
  // For now, this line is typically used once for a fresh RTC:
  // rtc.adjust(DateTime(F(__DATE__), F(__TIME__))); // Uncomment this line if your RTC continually loses time or on first flash.

  if (!bmp.begin(0x76)) // Check if BMP280 initialized successfully at 0x76
  {
    lcd.setCursor(0, 1);
    lcd.print("BMP280 Fail 0x76 ");
    // Try common alternate address 0x77 if 0x76 fails
    if (!bmp.begin(0x77)) {
       lcd.setCursor(0, 1);
       lcd.print("BMP280 Fail 0x77 ");
       haltAndCatchFire(); // If both fail, halt
    }
  }
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                  Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                  Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                  Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                  Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */


  lcd.print("                "); // Clear the "System Start" second line
  lcd.setCursor(0,1);
  lcd.print("Sensors OK!");
  delay(1500); // Brief confirmation
  lcd.clear();


  playStartupChime();
}

void loop()
{
  unsigned long nowMillis = millis();
  DateTime now = rtc.now();

  // --- Imperial Conversions ---
  float tempC = bmp.readTemperature();
  currentTempF = (tempC * 9.0 / 5.0) + 32.0; // Celsius to Fahrenheit

  float pressurePa = bmp.readPressure();
  currentPressureInHg = pressurePa / 3386.389; // Pascal to inHg

  // --- Lightning Sensor Handling ---
  if (digitalRead(lightningInt) == HIGH)
  {
    intVal = lightning.readInterruptReg();
    if (intVal == LIGHTNING_INT)
    {
      byte distanceKm = lightning.distanceToStorm();
      lastDistanceMiles = (int)round(distanceKm * 0.621371); // Convert km to miles
      lastEnergy = lightning.lightningEnergy(); // lastEnergy is now declared
      lastLightningTimeMillis = nowMillis; // Record timestamp of this strike

      if (lastDistanceMiles <= 10 && nowMillis - lastAlertMillis > alertCooldown)
      {
        alertStartMillis = nowMillis;
        alertActive = true;
        lastAlertMillis = nowMillis; // Update last alert time
        playAlertChime();
      }
    }
  }

  // --- Clear Lightning Stats after 1 Hour ---
  if (lastLightningTimeMillis > 0 && nowMillis - lastLightningTimeMillis > lightningClearCooldown) {
      lastDistanceMiles = -1; // Reset to indicate no recent lightning
      lastEnergy = -1;        // Reset energy
      lastLightningTimeMillis = 0; // Reset timestamp
  }


  // --- Display Logic ---
  if (alertActive)
  {
    if (nowMillis - alertStartMillis < 30000) // Alert lasts for 30 seconds
    {
      lcd.setCursor(0, 0);
      lcd.print("!!! LIGHTNING !!!");
      lcd.setCursor(0, 1);
      lcd.print("Within ");
      lcd.print(lastDistanceMiles); // Display distance in miles
      lcd.print(" miles!    ");
      // Optional: Flash backlight during alert
      if ( (nowMillis / 500) % 2 == 0 ) lcd.backlight();
      else lcd.noBacklight();

    }
    else // Alert period ended
    {
      alertActive = false;
      lcd.backlight(); // Ensure backlight is on after alert
      lcd.clear();     // Clear screen after alert
      // Re-initialize main display elements if needed
      lcd.setCursor(0, 0);
      lcd.write((uint8_t)0); // WiFi icon
      displayClock(); // Refresh clock
      displayScrollingInfo(); // Refresh info
    }
  }
  else // Normal operation
  {
    lcd.setCursor(0, 0);
    lcd.write((uint8_t)0); // wifi icon

    if (now.minute() != lastClockMinute)
    {
      lastClockMinute = now.minute();
      displayClock();
    }

    if (nowMillis - lastCycle > 4000) // Cycle info every 4 seconds
    {
      lastCycle = nowMillis;
      displayScrollingInfo();
    }
  }
  delay(100); // Small delay to prevent CPU overload
}