#include <Wire.h>
#include <LiquidCrystal.h>

// LCD pin connections
const int rs = 12;
const int en = 11;
const int d4 = 10;
const int d5 = 9;
const int d6 = 8;
const int d7 = 7;

LiquidCrystal lcd(rs, en, d4, d5, d6, d7);

volatile int vibeValue = 0;

void setup() {
  Wire.begin(0x0A);           // ESP32 IS SLAVE at 0x0A
  Wire.onReceive(receiveEvent);
  Serial.begin(9600);

  lcd.begin(16, 2);           // initialize LCD (no .init or .backlight here)
  lcd.clear();
}

unsigned long lastUpdate = 0;

void loop() {
  unsigned long now = millis();

  if (now - lastUpdate >= 1000) {
    lastUpdate = now;

    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Rating Value ");
    lcd.print(vibeValue);

    lcd.setCursor(0, 1);
    lcd.print("   Submitted");

    Serial.println(vibeValue);
  }
}

void receiveEvent(int howMany) {
  if (Wire.available()) {
    vibeValue = Wire.read();   // read 1–5 sent by Arduino master
  }
}
