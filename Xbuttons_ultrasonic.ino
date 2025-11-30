#include <Wire.h>
#include <Servo.h>

#define OTHER_ARDUINO_ADDRESS 0x0A
#define MY_I2C_ADDRESS 0x08
#define ENABLE_ULTRASONIC true

// ========== PIN DEFINITIONS ==========
const int buttonPins[5] = {7, 6, 5, 4, 3};  // Buttons 1-5
const int servoPin = 10;
const int trigPin = 8;
const int echoPin = 9;

// ========== DATA STRUCTURES ==========
struct VibePacket {
  uint8_t db_percent;
  uint8_t bass_percent;
  uint8_t mids_percent;
  uint8_t highs_percent;
  uint8_t vibe_state;
  int8_t  db_delta;
} __attribute__((packed));

volatile VibePacket vibeData;

// ======= GLOBAL VARIABLES =======
volatile uint8_t lastButtonPressed = 0; // 1-5 or 0
volatile int lastDistance = 0;          // cm
Servo myServo;
int currentServoAngle = 90;

// ======= BUTTON DEBOUNCE =======
unsigned long lastDebounceTime[5] = {0,0,0,0,0};
const unsigned long debounceDelay = 50;
int buttonState[5] = {HIGH,HIGH,HIGH,HIGH,HIGH};
int lastButtonState[5] = {HIGH,HIGH,HIGH,HIGH,HIGH};

// ======= ULTRASONIC =======
#if ENABLE_ULTRASONIC
unsigned long lastUltrasonicRead = 0;
long duration;
int distance;
#endif

// ======= I2C COMMAND =======
volatile uint8_t i2cCommand = 0; // store last command from master

// ======= SETUP =======
void setup() {
  Serial.begin(115200);

  // Initialize I2C as SLAVE
  Wire.begin(MY_I2C_ADDRESS);
  Wire.onReceive(receiveEvent);
  Wire.onRequest(requestEvent);

  // Servo
  myServo.attach(servoPin);
  myServo.write(currentServoAngle);

  // Buttons
  for (int i=0;i<5;i++) pinMode(buttonPins[i], INPUT_PULLUP);

  // Ultrasonic
  #if ENABLE_ULTRASONIC
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  #endif

  Serial.print("Arduino I2C Slave Ready @ 0x");
  Serial.println(MY_I2C_ADDRESS, HEX);
}

// ======= LOOP =======
void loop() {
  handleButtons();

  #if ENABLE_ULTRASONIC
  handleUltrasonic();
  #endif

  delay(10); // Small delay to avoid flooding Serial
}

// ======= BUTTON HANDLER =======
void handleButtons() {
  for(int i=0;i<5;i++){
    int reading = digitalRead(buttonPins[i]);
    if(reading != lastButtonState[i]) lastDebounceTime[i] = millis();
    if((millis() - lastDebounceTime[i]) > debounceDelay){
      if(reading == LOW && buttonState[i] == HIGH){
        lastButtonPressed = i+1;
        Serial.print("Button ");
        Serial.print(i+1);
        Serial.println(" pressed");
        handleButtonAction(i+1);
      }
      buttonState[i] = reading;
    }
    lastButtonState[i] = reading;
  }
}

// ======= BUTTON ACTION =======
void handleButtonAction(int button){
  notifyOtherArduino(button);
  switch(button){
    case 1: moveServoTo(0); break;
    case 2: moveServoTo(45); break;
    case 3: moveServoTo(90); break;
    case 4: moveServoTo(135); break;
    case 5: moveServoTo(180); break;
  }
}

// ======= SERVO =======
void moveServoTo(int angle){
  angle = constrain(angle,0,180);
  currentServoAngle = angle;
  myServo.write(angle);
}

// ======= ULTRASONIC =======
#if ENABLE_ULTRASONIC
void handleUltrasonic(){
  unsigned long now = millis();
  if(now - lastUltrasonicRead >= 200){
    lastUltrasonicRead = now;

    digitalWrite(trigPin, LOW); delayMicroseconds(2);
    digitalWrite(trigPin, HIGH); delayMicroseconds(10);
    digitalWrite(trigPin, LOW);

    duration = pulseIn(echoPin,HIGH);
    distance = duration*0.0343/2;
    lastDistance = distance;

    Serial.print("Distance: "); Serial.print(distance); Serial.println(" cm");
  }
}
#endif

// Called when ESP32 sends a byte (command)
void receiveEvent(int howMany){
  if(howMany >= 1){
    i2cCommand = Wire.read(); // 1 = button, 2 = distance
    while(Wire.available()) Wire.read(); // flush extra
  }
}

// Called when ESP32 requests data
void requestEvent(){
  if(i2cCommand == 1){
    Wire.write(lastButtonPressed);
    lastButtonPressed = 0;
  }
  else if(i2cCommand == 2){
    Wire.write((uint8_t)(lastDistance & 0xFF));        // LSB
    Wire.write((uint8_t)((lastDistance >> 8) & 0xFF)); // MSB
  }
  i2cCommand = 0; // reset
}

// ======= NOTIFY OTHER ARDUINO =======
void notifyOtherArduino(uint8_t button){
  Wire.beginTransmission(OTHER_ARDUINO_ADDRESS);
  Wire.write(button);
  Wire.endTransmission();
  Serial.print("Sent button "); Serial.print(button);
  Serial.println(" to Arduino @0x0A");
}
