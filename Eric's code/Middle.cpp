#include <Wire.h>
#include <LiquidCrystal_I2C.h>

LiquidCrystal_I2C lcd(0x27, 16, 2);

volatile int vibeValue = 0;   // number 1–5 received

void setup() {
  Wire.begin(10);                 // slave address
  Wire.onReceive(receiveEvent);
  Serial.begin(9600);

  lcd.init();
  lcd.begin(16, 2);
  lcd.backlight();
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
    vibeValue = Wire.read();   // read 1–5
  }
}