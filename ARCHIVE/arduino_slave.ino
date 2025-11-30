#include <Wire.h>
#include <Servo.h>

// ========== CONFIGURATION - CHANGE THIS FOR EACH ARDUINO ==========
#define MY_I2C_ADDRESS 0x08  // Change to 0x09, 0x0A for other Arduinos

// ========== PIN DEFINITIONS ==========
const int buttonPins[5] = {7, 6, 5, 4, 3};  // Buttons 1-5
const int servoPin = 10;

// Optional: Ultrasonic sensor (only on one Arduino)
#define ENABLE_ULTRASONIC false  // Set to true on one Arduino if needed
const int trigPin = 8;
const int echoPin = 9;

// ========== DATA STRUCTURES ==========
struct VibePacket {
  uint8_t db_percent;      // 0-100
  uint8_t bass_percent;    // 0-100
  uint8_t mids_percent;    // 0-100
  uint8_t highs_percent;   // 0-100
  uint8_t vibe_state;      // 0-4 (SILENT to PARTY)
  int8_t  db_delta;        // -128 to +127
} __attribute__((packed));

volatile VibePacket vibeData;
volatile uint8_t lastButtonPressed = 0;  // Sent to ESP32 when requested

// ========== SERVO CONTROL ==========
Servo myServo;
int currentServoAngle = 90;  // Start at center

// ========== BUTTON DEBOUNCE ==========
unsigned long lastDebounceTime[5] = {0, 0, 0, 0, 0};
const unsigned long debounceDelay = 50;
int buttonState[5] = {HIGH, HIGH, HIGH, HIGH, HIGH};
int lastButtonState[5] = {HIGH, HIGH, HIGH, HIGH, HIGH};

// ========== ULTRASONIC (OPTIONAL) ==========
#if ENABLE_ULTRASONIC
  long duration;
  int distance;
  unsigned long lastUltrasonicRead = 0;
#endif

// ========== SETUP ==========
void setup() {
  Serial.begin(115200);

  // Initialize I2C as SLAVE
  Wire.begin(MY_I2C_ADDRESS);
  Wire.onReceive(receiveEvent);  // When ESP32 sends data
  Wire.onRequest(requestEvent);  // When ESP32 requests data

  // Setup servo
  myServo.attach(servoPin);
  myServo.write(currentServoAngle);

  // Setup buttons
  for (int i = 0; i < 5; i++) {
    pinMode(buttonPins[i], INPUT_PULLUP);
  }

  // Setup ultrasonic (if enabled)
  #if ENABLE_ULTRASONIC
    pinMode(trigPin, OUTPUT);
    pinMode(echoPin, INPUT);
  #endif

  Serial.print("Arduino I2C Slave Ready @ 0x");
  Serial.println(MY_I2C_ADDRESS, HEX);
  Serial.println("Waiting for ESP32 vibe data...\n");
}

// ========== MAIN LOOP ==========
void loop() {
  handleButtons();

  #if ENABLE_ULTRASONIC
    handleUltrasonic();
  #endif

  delay(10);
}

// ========== BUTTON HANDLER ==========
void handleButtons() {
  for (int i = 0; i < 5; i++) {
    int reading = digitalRead(buttonPins[i]);

    if (reading != lastButtonState[i]) {
      lastDebounceTime[i] = millis();
    }

    if ((millis() - lastDebounceTime[i]) > debounceDelay) {
      if (reading == LOW && buttonState[i] == HIGH) {
        // Button pressed!
        lastButtonPressed = i + 1;  // Store 1-5

        Serial.print("Button ");
        Serial.print(i + 1);
        Serial.println(" pressed");

        // Custom action based on button
        handleButtonAction(i + 1);
      }
      buttonState[i] = reading;
    }
    lastButtonState[i] = reading;
  }
}

// ========== BUTTON ACTION HANDLER ==========
void handleButtonAction(int button) {
  // Example: Different servo positions for each button
  switch (button) {
    case 1:
      moveServoTo(0);
      Serial.println("  → Servo to 0°");
      break;
    case 2:
      moveServoTo(45);
      Serial.println("  → Servo to 45°");
      break;
    case 3:
      moveServoTo(90);
      Serial.println("  → Servo to 90°");
      break;
    case 4:
      moveServoTo(135);
      Serial.println("  → Servo to 135°");
      break;
    case 5:
      moveServoTo(180);
      Serial.println("  → Servo to 180°");
      break;
  }
}

// ========== ULTRASONIC HANDLER ==========
#if ENABLE_ULTRASONIC
void handleUltrasonic() {
  unsigned long now = millis();
  if (now - lastUltrasonicRead >= 1000) {  // Every 1 second
    lastUltrasonicRead = now;

    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);

    duration = pulseIn(echoPin, HIGH);
    distance = duration * 0.0343 / 2;

    Serial.print("Distance: ");
    Serial.print(distance);
    Serial.println(" cm");
  }
}
#endif

// ========== SERVO CONTROL ==========
void moveServoTo(int angle) {
  angle = constrain(angle, 0, 180);
  currentServoAngle = angle;
  myServo.write(angle);
}

void updateServoFromVibe() {
  // Map dB level (0-100) to servo angle (0-180)
  int targetAngle = map(vibeData.db_percent, 0, 100, 0, 180);
  moveServoTo(targetAngle);
}

// ========== I2C RECEIVE CALLBACK (ESP32 sends vibe data) ==========
void receiveEvent(int howMany) {
  if (howMany < sizeof(VibePacket)) {
    // Flush incomplete data
    while (Wire.available()) Wire.read();
    return;
  }

  // Read struct byte-by-byte
  uint8_t* ptr = (uint8_t*)&vibeData;
  for (int i = 0; i < sizeof(VibePacket) && Wire.available(); i++) {
    ptr[i] = Wire.read();
  }

  // Flush extra bytes
  while (Wire.available()) Wire.read();

  // Print received vibe data
  Serial.print("ESP32 → Vibe=");
  Serial.print(vibeData.vibe_state);
  Serial.print(" | dB=");
  Serial.print(vibeData.db_percent);
  Serial.print("% | B=");
  Serial.print(vibeData.bass_percent);
  Serial.print(" | M=");
  Serial.print(vibeData.mids_percent);
  Serial.print(" | H=");
  Serial.print(vibeData.highs_percent);
  Serial.print(" | Δ=");
  Serial.println(vibeData.db_delta);

  // Update servo based on vibe level
  updateServoFromVibe();
}

// ========== I2C REQUEST CALLBACK (ESP32 requests button data) ==========
void requestEvent() {
  // Send last button pressed (0 if none, 1-5 if pressed)
  Wire.write(lastButtonPressed);

  // Reset after sending
  if (lastButtonPressed != 0) {
    Serial.print("Sent button ");
    Serial.print(lastButtonPressed);
    Serial.println(" to ESP32");
    lastButtonPressed = 0;
  }
}
