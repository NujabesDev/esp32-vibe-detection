#include <Wire.h>

const int buttonPin1 = 7; // 1
const int buttonPin2 = 6; // 2
const int buttonPin3 = 5; // 3
const int buttonPin4 = 4; // 4
const int buttonPin5 = 3; // 5
const int trigPin = 8;
const int echoPin = 9;

long duration;
int distance;
unsigned long lastUpdate = 0;


struct VibePacket {
  uint8_t db_percent;
  uint8_t bass_percent;
  uint8_t mids_percent;
  uint8_t highs_percent;
  uint8_t vibe_state;
  int8_t  db_delta;
} __attribute__((packed));

VibePacket packetLeft;
VibePacket packetRight;
VibePacket packetMiddle;

void setup() {
  Wire.begin();
  Serial.begin(9600);

  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

  pinMode(buttonPin1, INPUT_PULLUP);
  pinMode(buttonPin2, INPUT_PULLUP);
  pinMode(buttonPin3, INPUT_PULLUP);
  pinMode(buttonPin4, INPUT_PULLUP);
  pinMode(buttonPin5, INPUT_PULLUP);

  packetLeft  = {50, 20, 30, 40, 1, -5};
  packetRight = {40, 60, 70, 80, 2,  3};
  packetMiddle = {90, 10, 20, 30, 3, -2};
}

// Debounce variables
unsigned long lastDebounceTime[5] = {0, 0, 0, 0, 0};
const unsigned long debounceDelay = 50; // ms
int buttonState[5] = {HIGH, HIGH, HIGH, HIGH, HIGH};
int lastButtonState[5] = {HIGH, HIGH, HIGH, HIGH, HIGH};
int buttonPins[5] = {buttonPin1, buttonPin2, buttonPin3, buttonPin4, buttonPin5};

void loop() {
  for (int i = 0; i < 5; i++) {
    int reading = digitalRead(buttonPins[i]);
    if (reading != lastButtonState[i]) {
      lastDebounceTime[i] = millis();
    }

    if ((millis() - lastDebounceTime[i]) > debounceDelay) {
      if (reading == LOW && buttonState[i] == HIGH) { // Button pressed
        Serial.print("Button ");
        Serial.print(i + 1);
        Serial.println(" pressed");


        sendNumberToArduino10(i + 1);
      }
      buttonState[i] = reading;
    }
    lastButtonState[i] = reading;
  }

  //-------- ULTRASOUND ----------
  unsigned long now = millis();
  if (now - lastUpdate >= 1000) { // 1 second interval
    lastUpdate = now;

    // Send a 10Âµs pulse
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);

    // Read echo pulse
    duration = pulseIn(echoPin, HIGH);

    // Calculate distance in cm
    distance = duration * 0.0343 / 2;

    // Print result
    Serial.print("Distance: ");
    Serial.print(distance);
    Serial.println(" cm");
  }
  //------------------------------

  // Optionally keep sending packets as before
  sendPacket(8,  packetLeft);   
  sendPacket(9,  packetRight);  
  // sendPacket(10, packetMiddle); 
}

void sendNumberToArduino10(uint8_t number) {
  Wire.beginTransmission(10);
  Wire.write(number);
  Wire.endTransmission();
}

void sendPacket(byte address, const VibePacket &p) {
  Wire.beginTransmission(address);
  Wire.write((uint8_t*)&p, sizeof(VibePacket));
  Wire.endTransmission();
}