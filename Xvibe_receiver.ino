/*
 * ESP32 Vibe Receiver - ESP-NOW + I2C
 * Receives vibe packets and forwards to Arduino via I2C
 */

#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <Wire.h>
#include <TFT_eSPI.h>
#include <NimBLEDevice.h>

// ========== CONFIGURATION ==========
#define I2C_SDA_PIN 21
#define I2C_SCL_PIN 22
#define ENABLE_I2C_FORWARD true

// Multiple Arduino slave addresses
#define ARDUINO_1_ADDRESS 0x08
#define ARDUINO_2_ADDRESS 0x09
const uint8_t arduinoAddresses[] = {ARDUINO_1_ADDRESS, ARDUINO_2_ADDRESS};
const int numArduinos = 2;

// ========== DISPLAY CONFIGURATION ==========
// TFT Display Colors
#define C_TEXT_MAIN    TFT_WHITE
#define C_BG           TFT_BLACK
#define C_LABEL        TFT_SILVER  // Grey for labels
#define C_VAL_DB       0x07FF      // Cyan
#define C_VAL_POS      TFT_GREEN   // Green for good vibes/+
#define C_VAL_NEG      TFT_RED     // Red for bad/-
#define C_VAL_ACCENT   0xFFE0      // Yellow
#define C_BASS         0xF200      // Red/Orange
#define C_MIDS         0xFFE0      // Yellow
#define C_HIGHS        0x06FF      // Blue
#define C_GRID         0x18E3      // Very dark grey

const int SCREEN_W = 480;
const int SCREEN_H = 320;

// Screen power control
const uint16_t SCREEN_ON_DISTANCE = 50;  // Turn screen on if < 50cm

// Human sentiment tracking
#define MAX_HUMAN_SAMPLES 100
#define MAX_DEVICES 8

// Spectrum analyzer
const int NUM_BARS = 32;
const int SPECTRUM_Y = 250;     // Bottom of bars
const int SPECTRUM_H = 150;     // Max height

// ========== DATA STRUCTURES ==========
enum VibeState {
  SILENT = 0,
  AMBIENT = 1,
  CONVERSATION = 2,
  ACTIVE = 3,
  PARTY = 4
};

struct VibePacket {
  uint8_t db_percent;
  uint8_t bass_percent;
  uint8_t mids_percent;
  uint8_t highs_percent;
  uint8_t vibe_state;
  int8_t db_delta;
} __attribute__((packed));

// ========== GLOBAL VARIABLES ==========
VibePacket lastPacket;
unsigned long lastReceiveTime = 0;
unsigned long packetCount = 0;
unsigned long lastErrorPrint = 0;

// Display objects
TFT_eSPI tft = TFT_eSPI();
NimBLEScan* bleScanner;

// Display state
int deviceCount = 0;
int bleCount = 0;
uint16_t ultrasonicDistance = 0;
bool screenOn = true;

// State caching (prevents flicker)
int prevDB = -1;
int prevDelta = -999;
int prevVibeState = -1;
int prevHumanVibe = -1;
int prevDevices = -1;

// Human sentiment tracking
uint8_t humanSamples[MAX_HUMAN_SAMPLES];
uint8_t humanIndex = 0;
uint8_t numHumanSamples = 0;

// Device tracking (BLE)
uint8_t knownMacs[MAX_DEVICES][6];

// Spectrum analyzer
float currentBarHeight[NUM_BARS] = {0};
int targetBarHeight[NUM_BARS] = {0};
int prevRenderedHeight[NUM_BARS] = {0};

// ========== FUNCTIONS ==========
const char* vibeStateToString(VibeState state) {
  switch (state) {
    case SILENT: return "SILENT";
    case AMBIENT: return "AMBIENT";
    case CONVERSATION: return "CONVERSATION";
    case ACTIVE: return "ACTIVE";
    case PARTY: return "PARTY";
    default: return "UNKNOWN";
  }
}

// ========== UTILITY FUNCTIONS ==========

void registerDevice(const uint8_t *mac) {
  for (int i = 0; i < deviceCount; i++) {
    bool same = true;
    for (int b = 0; b < 6; b++) if (knownMacs[i][b] != mac[b]) same = false;
    if (same) return;
  }
  if (deviceCount < MAX_DEVICES) {
    memcpy(knownMacs[deviceCount], mac, 6);
    deviceCount++;
  }
}

uint16_t getBarColor(int i) {
  if (i < 10) return C_BASS;
  if (i < 21) return C_MIDS;
  return C_HIGHS;
}

// ========== HUMAN SENTIMENT TRACKING ==========

void addHumanVibeSample(uint8_t btnVal) {
  if (btnVal < 1 || btnVal > 5) return;
  humanSamples[humanIndex] = btnVal;
  humanIndex = (humanIndex + 1) % MAX_HUMAN_SAMPLES;
  if (numHumanSamples < MAX_HUMAN_SAMPLES)
    numHumanSamples++;
}

float getAverageHumanVibe() {
  if (numHumanSamples == 0) return 0;
  int sum = 0;
  for (int i = 0; i < numHumanSamples; i++)
    sum += humanSamples[i];
  return (float)sum / numHumanSamples;
}

uint8_t getHumanVibeState() {
  float avg = getAverageHumanVibe();
  if (avg < 1.5) return 0;   // 1.x → SILENT
  if (avg < 2.5) return 1;   // 2.x → AMBIENT
  if (avg < 3.5) return 2;   // 3.x → CONVO
  if (avg < 4.5) return 3;   // 4.x → ACTIVE
  return 4;                  // 5.x → PARTY
}

// ========== BLE FUNCTIONS ==========

int scanBLEDevices() {
  NimBLEScanResults results = bleScanner->getResults();
  int count = results.getCount();
  bleScanner->clearResults();
  bleScanner->start(0, true, true);
  return count;
}

// ========== DRAWING FUNCTIONS ==========

void drawStaticLayout() {
  tft.fillScreen(C_BG);

  // Visual separators
  tft.drawFastHLine(0, 90, SCREEN_W, C_GRID);
  tft.drawFastHLine(0, 260, SCREEN_W, C_GRID);

  // Static labels
  tft.setTextFont(2);
  tft.setTextSize(2);
  tft.setTextColor(C_LABEL, C_BG);

  tft.drawString("dB LEVEL", 15, 10);

  tft.setTextDatum(TR_DATUM);
  tft.drawString("HUMAN SENTIMENT", 465, 10);

  // Spectrum labels
  tft.setTextSize(1);
  tft.setTextDatum(TC_DATUM);
  tft.drawString("BASS", 80, 265);
  tft.drawString("MIDS", 240, 265);
  tft.drawString("HIGHS", 400, 265);
  tft.setTextSize(2);

  // Bottom stats labels
  tft.setTextDatum(TL_DATUM);
  tft.drawString("DEVICES", 20, 290);

  tft.setTextSize(1);
  tft.setTextDatum(TR_DATUM);
  tft.drawString("COMPUTER SENTIMENT -->", 350, 300);
}

void updateTopDashboard() {
  tft.setTextSize(1);

  // 1. dB LEVEL with real db_delta
  if (lastPacket.db_percent != prevDB || lastPacket.db_delta != prevDelta) {
    prevDB = lastPacket.db_percent;
    prevDelta = lastPacket.db_delta;

    tft.setTextFont(6);
    tft.setTextDatum(TL_DATUM);
    tft.setTextColor(C_VAL_DB, C_BG);

    int x = tft.drawNumber(lastPacket.db_percent, 15, 40);

    // Draw real db_delta from packet
    tft.setTextFont(4);
    String dStr = (lastPacket.db_delta >= 0 ? "+" : "") + String(lastPacket.db_delta);
    uint16_t dCol = (lastPacket.db_delta >= 0 ? C_VAL_POS : C_VAL_NEG);
    tft.setTextColor(dCol, C_BG);
    tft.fillRect(75, 50, 70, 30, C_BG);
    tft.drawString(dStr, 75, 50);
  }

  // 2. Human sentiment
  uint8_t humanVibeState = getHumanVibeState();
  if (humanVibeState != prevHumanVibe) {
    prevHumanVibe = humanVibeState;

    const char* states[] = {"SILENT", "AMBIENT", "CONVO", "ACTIVE", "PARTY"};

    tft.setTextFont(4);
    tft.setTextDatum(TR_DATUM);
    tft.setTextColor(C_VAL_ACCENT, C_BG);

    tft.fillRect(220, 45, 260, 40, C_BG);
    tft.drawString(states[humanVibeState], 465, 45);
  }
}

void updateBottomStats() {
  tft.setTextSize(1);

  // 1. Device count
  if (deviceCount != prevDevices) {
    prevDevices = deviceCount;
    tft.setTextFont(4);
    tft.setTextDatum(TL_DATUM);
    tft.setTextColor(C_TEXT_MAIN, C_BG);
    tft.fillRect(140, 285, 40, 30, C_BG);
    tft.drawNumber(deviceCount, 140, 295);
  }

  // 2. Computer sentiment (from VibePacket vibe_state)
  if (lastPacket.vibe_state != prevVibeState) {
    prevVibeState = lastPacket.vibe_state;

    const char* states[] = {"SILENT", "AMBIENT", "CONVO", "ACTIVE", "PARTY"};
    uint8_t s = (lastPacket.vibe_state > 4) ? 0 : lastPacket.vibe_state;

    tft.setTextFont(4);
    tft.setTextDatum(TR_DATUM);
    tft.setTextColor(C_VAL_ACCENT, C_BG);

    tft.fillRect(350, 290, 150, 35, C_BG);
    tft.drawString(states[s], 465, 295);
  }
}

void updateSpectrum() {
  // Map incoming 3-band data to 32 bars with noise
  for (int i = 0; i < NUM_BARS; i++) {
    int target = 0;
    int noise = random(-3, 4);

    if (i < 10) { // BASS
      target = lastPacket.bass_percent;
      float curve = 1.0 - (abs(i - 5) / 6.0);
      target = (target * curve) + noise;
    } else if (i < 22) { // MIDS
      target = lastPacket.mids_percent;
      float curve = 1.0 - (abs(i - 16) / 8.0);
      target = (target * curve) + noise;
    } else { // HIGHS
      target = lastPacket.highs_percent;
      float curve = 1.0 - (abs(i - 27) / 6.0);
      target = (target * curve) + noise;
    }
    targetBarHeight[i] = constrain(target, 2, 100);
  }
}

void drawSpectrumLoop() {
  int barW = 10;
  int gap = 4;
  int startX = 20;

  for (int i = 0; i < NUM_BARS; i++) {
    // Animation smoothing
    float diff = targetBarHeight[i] - currentBarHeight[i];
    currentBarHeight[i] += diff * 0.25;

    int h = map((int)currentBarHeight[i], 0, 100, 0, SPECTRUM_H);

    if (h != prevRenderedHeight[i]) {
      int x = startX + i * (barW + gap);
      int oldH = prevRenderedHeight[i];
      uint16_t color = getBarColor(i);

      if (h > oldH) {
        tft.fillRect(x, SPECTRUM_Y - h, barW, h - oldH, color);
      } else {
        tft.fillRect(x, SPECTRUM_Y - oldH, barW, oldH - h, C_BG);
      }
      prevRenderedHeight[i] = h;
    }
  }
}

void updateScreenPower() {
  bool shouldBeOn = (ultrasonicDistance > 0 && ultrasonicDistance < SCREEN_ON_DISTANCE);

  if (shouldBeOn && !screenOn) {
    tft.writecommand(0x29);  // Display ON
    screenOn = true;
    Serial.println("Screen ON");
  } else if (!shouldBeOn && screenOn) {
    tft.writecommand(0x28);  // Display OFF
    screenOn = false;
    Serial.println("Screen OFF");
  }
}

// ========== ESP-NOW CALLBACK ==========

void onDataRecv(const esp_now_recv_info_t *recv_info, const uint8_t *data, int data_len) {
  if (data_len != sizeof(VibePacket)) {
    Serial.println("ERROR: Packet size mismatch");
    return;
  }

  memcpy(&lastPacket, data, sizeof(VibePacket));
  lastReceiveTime = millis();
  packetCount++;

  // Update spectrum display targets
  updateSpectrum();

  // Forward to all Arduinos via I2C
  #if ENABLE_I2C_FORWARD
    for (int i = 0; i < numArduinos; i++) {
      Wire.beginTransmission(arduinoAddresses[i]);
      Wire.write((uint8_t*)&lastPacket, sizeof(VibePacket));
      uint8_t i2c_result = Wire.endTransmission();

      // Rate limit status printing to every 10 seconds
      if (millis() - lastErrorPrint >= 10000) {
        lastErrorPrint = millis();
        if (i2c_result == 0) {
          Serial.print("✓ Arduino 0x");
          Serial.print(arduinoAddresses[i], HEX);
          Serial.println(" OK");
        } else {
          Serial.print("✗ Arduino 0x");
          Serial.print(arduinoAddresses[i], HEX);
          Serial.print(" Error ");
          Serial.println(i2c_result);
        }
      }
    }
  #endif
}

// ========== SETUP ==========
void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10);

  Serial.println("\n=== ESP32 Vibe Display + Receiver ===\n");

  // Initialize TFT display
  tft.init();
  tft.setRotation(1);
  tft.fillScreen(C_BG);
  drawStaticLayout();
  Serial.println("✓ TFT display initialized");

  WiFi.mode(WIFI_STA);
  WiFi.begin();  // Initialize WiFi to get MAC
  delay(100);

  // Get MAC address using esp_wifi
  uint8_t baseMac[6];
  esp_wifi_get_mac(WIFI_IF_STA, baseMac);
  Serial.print("✓ MY MAC ADDRESS: ");
  Serial.printf("%02X:%02X:%02X:%02X:%02X:%02X\n",
                baseMac[0], baseMac[1], baseMac[2],
                baseMac[3], baseMac[4], baseMac[5]);
  Serial.println(">> Copy this to transmitter's receiverMAC[] array <<\n");

  if (esp_now_init() != ESP_OK) {
    Serial.println("ERROR: ESP-NOW init failed");
    return;
  }
  Serial.println("✓ ESP-NOW initialized");

  esp_now_register_recv_cb(onDataRecv);
  Serial.println("✓ Receiver callback registered");

  #if ENABLE_I2C_FORWARD
    Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);  // ESP32 as I2C Master
    Serial.println("✓ I2C Master initialized");
    Serial.print("  → Arduinos at: 0x");
    Serial.print(ARDUINO_1_ADDRESS, HEX);
    Serial.print(", 0x");
    Serial.println(ARDUINO_2_ADDRESS, HEX);
  #else
    Serial.println("✓ I2C forwarding disabled");
  #endif

  // Initialize BLE scanner
  NimBLEDevice::init("");
  bleScanner = NimBLEDevice::getScan();
  bleScanner->setActiveScan(true);
  bleScanner->setInterval(45);
  bleScanner->setWindow(30);
  bleScanner->start(0, true, true);  // Continuous background scan
  Serial.println("✓ BLE scanner initialized");

  Serial.println("\nSystem ready!\n");
}

// ========== REQUEST BUTTON DATA FROM ARDUINOS ==========
void requestButtonData() {
  // Tell Arduino we want BUTTON data
  Wire.beginTransmission(ARDUINO_1_ADDRESS);
  Wire.write(1);   // Command 1 = button
  Wire.endTransmission();

  // Now request 1 byte back
  Wire.requestFrom(ARDUINO_1_ADDRESS, 1);

  if (Wire.available()) {
    uint8_t buttonData = Wire.read();
    if (buttonData > 0 && buttonData <= 5) {
      addHumanVibeSample(buttonData);  // Track for human sentiment
      Serial.print("Button ");
      Serial.print(buttonData);
      Serial.println(" pressed");
    }
  }
}

void requestUltrasonicData() {
  // Tell Arduino we want DISTANCE data
  Wire.beginTransmission(ARDUINO_1_ADDRESS);
  Wire.write(2);  // Command 2 = distance
  Wire.endTransmission();

  // Now request 2 bytes (LSB, MSB)
  Wire.requestFrom(ARDUINO_1_ADDRESS, 2);

  if (Wire.available() >= 2) {
    uint8_t low  = Wire.read();
    uint8_t high = Wire.read();

    ultrasonicDistance = (high << 8) | low;  // Store for screen control

    if (ultrasonicDistance > 0 && ultrasonicDistance < 500) {
      Serial.print("Distance: ");
      Serial.print(ultrasonicDistance);
      Serial.println(" cm");
    }
  }
}


// ========== MAIN LOOP ==========
void loop() {
  static uint32_t lastI2CRequest = 0;
  static uint32_t lastScan = 0;

  if (packetCount > 0 && (millis() - lastReceiveTime) > 5000) {
    Serial.println("WARNING: No packets in 5 seconds");
    delay(1000);
  }

  // Request button and ultrasonic data from Arduino every 100ms
  #if ENABLE_I2C_FORWARD
    if (millis() - lastI2CRequest > 100) {
      lastI2CRequest = millis();
      requestButtonData();
      requestUltrasonicData();
      updateScreenPower();  // Control screen based on distance
    }
  #endif

  // BLE scan every 2 seconds
  if (millis() - lastScan > 2000) {
    lastScan = millis();
    bleCount = scanBLEDevices();
    deviceCount = bleCount;
  }

  // Update display (only if screen is on)
  if (screenOn) {
    updateTopDashboard();
    updateBottomStats();
    drawSpectrumLoop();
  }

  delay(10);
}
