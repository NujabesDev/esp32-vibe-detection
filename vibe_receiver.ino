/*
 * ESP32 Vibe Receiver - ESP-NOW + I2C
 * Receives vibe packets and forwards to Arduino via I2C
 */

#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <Wire.h>

// ========== CONFIGURATION ==========
#define I2C_SDA_PIN 21
#define I2C_SCL_PIN 22
#define ENABLE_I2C_FORWARD true

// Multiple Arduino slave addresses
#define ARDUINO_1_ADDRESS 0x08
#define ARDUINO_2_ADDRESS 0x09
// #define ARDUINO_3_ADDRESS 0x0A
const uint8_t arduinoAddresses[] = {ARDUINO_1_ADDRESS, ARDUINO_2_ADDRESS};
const int numArduinos = 2;

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

void onDataRecv(const esp_now_recv_info_t *recv_info, const uint8_t *data, int data_len) {
  if (data_len != sizeof(VibePacket)) {
    Serial.println("ERROR: Packet size mismatch");
    return;
  }

  memcpy(&lastPacket, data, sizeof(VibePacket));
  lastReceiveTime = millis();
  packetCount++;

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

  // Print data (commented out for debugging)
  // Serial.print("From: ");
  // for (int i = 0; i < 6; i++) {
  //   Serial.printf("%02X", recv_info->src_addr[i]);
  //   if (i < 5) Serial.print(":");
  // }
  // Serial.print(" | Pkt #");
  // Serial.println(packetCount);

  // Serial.print("Vibe: ");
  // Serial.print(vibeStateToString((VibeState)lastPacket.vibe_state));
  // Serial.print(" | dB=");
  // Serial.print(lastPacket.db_percent);
  // Serial.print("% B=");
  // Serial.print(lastPacket.bass_percent);
  // Serial.print(" M=");
  // Serial.print(lastPacket.mids_percent);
  // Serial.print(" H=");
  // Serial.print(lastPacket.highs_percent);
  // Serial.print(" Δ=");
  // Serial.println(lastPacket.db_delta);

  // // Visual bar
  // Serial.print("Volume: [");
  // int bars = map(lastPacket.db_percent, 0, 100, 0, 30);
  // for (int i = 0; i < bars; i++) Serial.print("=");
  // for (int i = bars; i < 30; i++) Serial.print(" ");
  // Serial.println("]");
  // Serial.println();
}

// ========== SETUP ==========
void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10);

  Serial.println("\n=== ESP32 Vibe Receiver ===\n");

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
    Serial.print(ARDUINO_2_ADDRESS, HEX);
    Serial.print(", 0x");
    // Serial.println(ARDUINO_3_ADDRESS, HEX);
  #else
    Serial.println("✓ I2C forwarding disabled");
  #endif

  Serial.println("\nWaiting for packets...\n");
}

// ========== REQUEST BUTTON DATA FROM ARDUINOS ==========
void requestButtonData() {
  for (int i = 0; i < numArduinos; i++) {
    Wire.requestFrom(arduinoAddresses[i], 1);  // Request 1 byte (button state)

    if (Wire.available()) {
      uint8_t buttonData = Wire.read();
      if (buttonData > 0) {  // If any button pressed
        Serial.print("Arduino 0x");
        Serial.print(arduinoAddresses[i], HEX);
        Serial.print(" - Button ");
        Serial.print(buttonData);
        Serial.println(" pressed");
      }
    }
  }
}

// ========== MAIN LOOP ==========
void loop() {
  if (packetCount > 0 && (millis() - lastReceiveTime) > 5000) {
    Serial.println("WARNING: No packets in 5 seconds");
    delay(1000);
  }

  // Request button data from all Arduinos every 100ms
  #if ENABLE_I2C_FORWARD
    // requestButtonData();  // Disabled - causes I2C NACK errors if Arduinos don't have onRequest handler
  #endif

  delay(100);
}