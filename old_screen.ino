
#include <WiFi.h>
#include <esp_now.h>
#include <TFT_eSPI.h>
#include <NimBLEDevice.h>
#include <Wire.h>

#define I2C_ADDR 0x21   // any address you and Arduino agree on

// HUMAN SENTIMENT button averaging
#define MAX_HUMAN_SAMPLES 100   // how many button presses to remember
uint8_t humanSamples[MAX_HUMAN_SAMPLES];
uint8_t humanIndex = 0;
uint8_t numHumanSamples = 0;
int     prevHumanVibe = -1;    // for UI caching


// bluetooth stuff
NimBLEScan* bleScanner;
int bleCount = 0;

// oled
TFT_eSPI tft = TFT_eSPI();

// ================= COLORS =================
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

// ================= DATA STRUCT =================
struct __attribute__((packed)) VibePacket {
  uint8_t db_percent;
  uint8_t bass_percent;
  uint8_t mids_percent;
  uint8_t highs_percent;
  uint8_t vibe_state;
  uint8_t vibe_score;
};

VibePacket currentPacket = {0,0,0,0,0,0};
int deviceCount = 0;

// ================= STATE CACHING =================
// We store previous values to only redraw what changes (prevents flicker)
int prevDB = -1;
int prevDelta = -999;
int prevVibeState = -1;
int prevScore = -1;
int prevDevices = -1;

// Spectrum Vars
const int NUM_BARS = 32;
float currentBarHeight[NUM_BARS] = {0};
int   targetBarHeight[NUM_BARS] = {0};
int   prevRenderedHeight[NUM_BARS] = {0};
const int SPECTRUM_Y = 250;     // Bottom of bars
const int SPECTRUM_H = 150;     // Max height

// ================= UTILS =================
#define MAX_DEVICES 8
uint8_t knownMacs[MAX_DEVICES][6];

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

// ================= DRAWING =================

void drawStaticLayout() {
  tft.fillScreen(C_BG);

  // --- VISUAL SEPARATORS ---
  // Thin faint lines to organize the screen
  tft.drawFastHLine(0, 90, SCREEN_W, C_GRID);
  tft.drawFastHLine(0, 260, SCREEN_W, C_GRID);

  // --- STATIC LABELS (Font 2 is small sans-serif) ---
  tft.setTextFont(2);
  tft.setTextSize(2);
  tft.setTextColor(C_LABEL, C_BG);

  // Top Left
  tft.drawString("dB LEVEL", 15, 10);

  // Top Right (Label aligned to match value)
  tft.setTextDatum(TR_DATUM);
  tft.drawString("HUMAN SENTIMENT", 465, 10);

  // Spectrum Labels
  tft.setTextSize(1);

  tft.setTextDatum(TC_DATUM); // Center align
  tft.drawString("BASS", 80, 265);
  tft.drawString("MIDS", 240, 265);
  tft.drawString("HIGHS", 400, 265);
  tft.setTextSize(2);

  // Bottom Stats Labels
  tft.setTextDatum(TL_DATUM); // Left align
  tft.drawString("DEVICES", 20, 290);

  tft.setTextSize(1);
  tft.setTextDatum(TR_DATUM); // Right align
  tft.drawString("COMPUTER SENTIMENT -->", 350, 300);
}

void updateTopDashboard() {
  tft.setTextSize(1); // Ensure size 1 for Fonts 4/6

  // 1. dB LEVEL (Big Number)
  if (currentPacket.db_percent != prevDB) {
    int delta = (prevDB == -1) ? 0 : (currentPacket.db_percent - prevDB);
    prevDB = currentPacket.db_percent;

    // Use Font 6 (Large Digits) if available, otherwise Font 4
    tft.setTextFont(6);
    tft.setTextDatum(TL_DATUM);
    tft.setTextColor(C_VAL_DB, C_BG); // Color, Background (erases old)

    // Draw "88%"
    // Font 6 is numbers only. We draw number then % sign manually
    int x = tft.drawNumber(currentPacket.db_percent, 15, 40);

    // Draw Delta Small next to it
    tft.setTextFont(4);
    String dStr = (delta >= 0 ? "+" : "") + String(delta);
    uint16_t dCol = (delta >= 0 ? C_VAL_POS : C_VAL_NEG);
    tft.setTextColor(dCol, C_BG);
    tft.drawString(dStr, 75, 50);
  }

  // 2. HUMAN SENTIMENT
  // if (currentPacket.vibe_state != prevVibeState) {
  //   prevVibeState = currentPacket.vibe_state;

  //   const char* states[] = {"SILENT", "AMBIENT", "CONVO", "MUSIC", "PARTY"};
  //   uint8_t s = (currentPacket.vibe_state > 4) ? 0 : currentPacket.vibe_state;

  //   tft.setTextFont(4);
  //   tft.setTextDatum(TR_DATUM); // Right Align
  //   tft.setTextColor(C_VAL_ACCENT, C_BG);

  //   // Clear area explicitly to be safe
  //   tft.fillRect(250, 45, 220, 35, C_BG);
  //   tft.drawString(states[s], 465, 45);
  // }
  uint8_t humanVibeState = getHumanVibeState();

  if (humanVibeState != prevHumanVibe) {
    prevHumanVibe = humanVibeState;

    const char* states[] = {"SILENT", "AMBIENT", "CONVO", "PARTY"};

    tft.setTextFont(4);
    tft.setTextDatum(TR_DATUM);
    tft.setTextColor(C_VAL_ACCENT, C_BG);

    // clear previous text
    tft.fillRect(220, 45, 260, 40, C_BG);

    // draw sentiment name
    tft.drawString(states[humanVibeState], 465, 45);
  }
}

void updateBottomStats() {
  tft.setTextSize(1);

  // 1. DEVICES
  if (deviceCount != prevDevices) {
    prevDevices = deviceCount;
    tft.setTextFont(4);
    tft.setTextDatum(TL_DATUM);
    tft.setTextColor(C_TEXT_MAIN, C_BG); // White
    tft.fillRect(140, 285, 40, 30, C_BG); // Clear spot
    tft.drawNumber(deviceCount, 140, 295);
  }

  // 2. computer sentiment
  if (currentPacket.vibe_state != prevVibeState) {
    prevVibeState = currentPacket.vibe_state;

    const char* states[] = {"SILENT", "AMBIENT", "CONVO", "MUSIC", "PARTY"};
    uint8_t s = (currentPacket.vibe_state > 4) ? 0 : currentPacket.vibe_state;

    tft.setTextFont(4);
    tft.setTextDatum(TR_DATUM); // Right Align
    tft.setTextColor(C_VAL_ACCENT, C_BG);

    // Clear area explicitly to be safe
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
      target = currentPacket.bass_percent;
      // Shape it: Peak in middle of bass section
      float curve = 1.0 - (abs(i - 5) / 6.0);
      target = (target * curve) + noise;
    } else if (i < 22) { // MIDS
      target = currentPacket.mids_percent;
      float curve = 1.0 - (abs(i - 16) / 8.0);
      target = (target * curve) + noise;
    } else { // HIGHS
      target = currentPacket.highs_percent;
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
    // Animation Smoothing
    float diff = targetBarHeight[i] - currentBarHeight[i];
    currentBarHeight[i] += diff * 0.25; // Speed

    int h = map((int)currentBarHeight[i], 0, 100, 0, SPECTRUM_H);

    if (h != prevRenderedHeight[i]) {
      int x = startX + i * (barW + gap);
      int oldH = prevRenderedHeight[i];
      uint16_t color = getBarColor(i);

      if (h > oldH) {
        // Grow: Draw color
        tft.fillRect(x, SPECTRUM_Y - h, barW, h - oldH, color);
      } else {
        // Shrink: Draw black
        tft.fillRect(x, SPECTRUM_Y - oldH, barW, oldH - h, C_BG);
      }
      prevRenderedHeight[i] = h;
    }
  }
}

// store one button value coming from Arduino
// btnVal is expected to be 1–4, we map to 0–3
void addHumanVibeSample(uint8_t btnVal) {
  // btnVal is 1–4 exactly from your buttons
  if (btnVal < 1 || btnVal > 4) return;

  humanSamples[humanIndex] = btnVal;  // <-- store RAW button value (1–4)
  humanIndex = (humanIndex + 1) % MAX_HUMAN_SAMPLES;

  if (numHumanSamples < MAX_HUMAN_SAMPLES)
    numHumanSamples++;
}


// return the average of all stored button inputs (0–3)
float getAverageHumanVibe() {
  if (numHumanSamples == 0) return 0;

  int sum = 0;
  for (int i = 0; i < numHumanSamples; i++)
    sum += humanSamples[i];

  return (float)sum / numHumanSamples;   // real average (ex: 2.4)
}

uint8_t getHumanVibeState() {
  float avg = getAverageHumanVibe();
  if (avg < 1.5) return 0;   // 1.x → SILENT
  if (avg < 2.5) return 1;   // 2.x → AMBIENT
  if (avg < 3.5) return 2;   // 3.x → CONVO
  return 3;                  // 4.x → PARTY
}


void onI2CReceive(int len) {
  while (Wire.available()) {
    uint8_t btnVal = Wire.read();   // 1–4 from Arduino
    addHumanVibeSample(btnVal);
  }
}


int scanBLEDevices() {
    // get current scan snapshot
    NimBLEScanResults results = bleScanner->getResults();
    int count = results.getCount();

    // prepare for next scan window
    bleScanner->clearResults();
    bleScanner->start(0, true, true);  // continuous async scan

    return count;
}


// ================= SETUP & LOOP =================

void OnDataRecv(const esp_now_recv_info *info, const uint8_t *data, int len) {
  if (len < sizeof(VibePacket)) return;
  memcpy(&currentPacket, data, sizeof(VibePacket));
  if (info->src_addr) registerDevice(info->src_addr);
  updateSpectrum(); // Update targets
}

void setup() {
  Serial.begin(115200);
  tft.init();
  tft.setRotation(1);
  tft.fillScreen(C_BG);

  drawStaticLayout();

  WiFi.mode(WIFI_STA);
  if (esp_now_init() == ESP_OK) {
    esp_now_register_recv_cb(OnDataRecv);
  }

  // --- I2C setup (ESP32 side) ---
  Wire.begin(I2C_ADDR);          // ESP32 listens on 0x21
  Wire.onReceive(onI2CReceive);  // callback when Arduino sends a byte

  //arduino side for button input
  // Wire.beginTransmission(0x21);  // same address as I2C_ADDR
  // Wire.write(buttonNumber);      // 1,2,3,4
  // Wire.endTransmission();


  // bluetooth setup
  NimBLEDevice::init("");
  bleScanner = NimBLEDevice::getScan();
  bleScanner->setActiveScan(true);
  bleScanner->setInterval(45);
  bleScanner->setWindow(30);

  // Start continuous background scan
  bleScanner->start(0, true, true);  // <--- IMPORTANT
}

void loop() {
  static uint32_t lastScan = 0;
  // SIMULATION -- Remove in production
  static uint32_t t = 0;
  if (millis() - t > 150) {
    t = millis();
    currentPacket.db_percent = random(60, 95);
    currentPacket.bass_percent = random(30, 90);
    currentPacket.mids_percent = random(20, 80);
    currentPacket.highs_percent = random(10, 70);
    currentPacket.vibe_state = random(0, 4);
    currentPacket.vibe_score = random(85, 99);
    updateSpectrum();
  }

  if (millis() - lastScan > 2000) {
    lastScan = millis();
    bleCount = scanBLEDevices();
    deviceCount = bleCount;
  }

  // 🔥 NEW: SERIAL TEST FOR HUMAN SENTIMENT BUTTONS 🔥
  // Type 1,2,3,4 into Serial Monitor and press Send
  if (Serial.available()) {
    char c = Serial.read();
    if (c >= '1' && c <= '4') {
      uint8_t btnVal = c - '0';     // '1'→1, '2'→2, '3'→3, '4'→4
      addHumanVibeSample(btnVal);

      Serial.print("Received human button: ");
      Serial.println(btnVal);
      Serial.print("New average vibe: ");
      Serial.println(getAverageHumanVibe());
    }
  }

  updateTopDashboard();
  updateBottomStats();
  drawSpectrumLoop();
  delay(10);
}
