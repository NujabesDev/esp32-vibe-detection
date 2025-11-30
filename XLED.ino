#include <Wire.h>

struct VibePacket {
  uint8_t db_percent;
  uint8_t bass_percent;
  uint8_t mids_percent;
  uint8_t highs_percent;
  uint8_t vibe_state;
  int8_t  db_delta;
} __attribute__((packed));

volatile VibePacket pkt;

// LED pins for 5-level meter
const int ledPins[5] = {8, 9, 10, 11, 12};

unsigned long lastUpdate = 0;
const unsigned long updateInterval = 200;

void setup() {
  Serial.begin(9600);

  Wire.begin(9);            // <-- LED SLAVE ADDRESS
  Wire.onReceive(receiveEvent);

  // Initialize LED pins
  for (int i = 0; i < 5; i++) {
    pinMode(ledPins[i], OUTPUT);
    digitalWrite(ledPins[i], LOW);
  }
}

void loop() {
  unsigned long now = millis();

  if (now - lastUpdate >= updateInterval) {
    lastUpdate = now;

    int value = pkt.db_percent;

    Serial.print("LED Slave dB%: ");
    Serial.println(value);

    // compute 0–4 level
    int level = value / 25;
    if (level > 4) level = 4;

    // turn on LEDs up to "level"
    for (int i = 0; i < 5; i++) {
      digitalWrite(ledPins[i], (i <= level) ? HIGH : LOW);
    }
  }
}

void receiveEvent(int howMany) {
  if (howMany == sizeof(VibePacket)) {
    Wire.readBytes((char*)&pkt, sizeof(VibePacket));
  } else {
    while (Wire.available()) Wire.read();
  }
}
