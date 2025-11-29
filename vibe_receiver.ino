/*
 * ESP32 Vibe Receiver - ESP-NOW + I2C
 * Receives vibe packets and forwards to Arduino via I2C
 */

#include <WiFi.h>
#include <esp_now.h>
#include <Wire.h>

// ========== CONFIGURATION ==========
#define I2C_SDA_PIN 21
#define I2C_SCL_PIN 22
#define ARDUINO_I2C_ADDRESS 0x08
#define ENABLE_I2C_FORWARD true

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

  // Forward to Arduino via I2C
  #if ENABLE_I2C_FORWARD
    Wire.beginTransmission(ARDUINO_I2C_ADDRESS);
    Wire.write((uint8_t*)&lastPacket, sizeof(VibePacket));
    uint8_t i2c_result = Wire.endTransmission();
    if (i2c_result != 0) {
      Serial.print("  [I2C Error ");
      Serial.print(i2c_result);
      Serial.println("]");
    }
  #endif

  // Print data
  Serial.print("From: ");
  for (int i = 0; i < 6; i++) {
    Serial.printf("%02X", recv_info->src_addr[i]);
    if (i < 5) Serial.print(":");
  }
  Serial.print(" | Pkt #");
  Serial.println(packetCount);

  Serial.print("Vibe: ");
  Serial.print(vibeStateToString((VibeState)lastPacket.vibe_state));
  Serial.print(" | dB=");
  Serial.print(lastPacket.db_percent);
  Serial.print("% B=");
  Serial.print(lastPacket.bass_percent);
  Serial.print(" M=");
  Serial.print(lastPacket.mids_percent);
  Serial.print(" H=");
  Serial.print(lastPacket.highs_percent);
  Serial.print(" Δ=");
  Serial.println(lastPacket.db_delta);

  // Visual bar
  Serial.print("Volume: [");
  int bars = map(lastPacket.db_percent, 0, 100, 0, 30);
  for (int i = 0; i < bars; i++) Serial.print("=");
  for (int i = bars; i < 30; i++) Serial.print(" ");
  Serial.println("]");
  Serial.println();
}

// ========== SETUP ==========
void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10);

  Serial.println("\n=== ESP32 Vibe Receiver ===\n");

  WiFi.mode(WIFI_STA);
  Serial.print("✓ MY MAC ADDRESS: ");
  Serial.println(WiFi.macAddress());
  Serial.println(">> Copy this to transmitter's receiverMAC[] array <<\n");

  if (esp_now_init() != ESP_OK) {
    Serial.println("ERROR: ESP-NOW init failed");
    return;
  }
  Serial.println("✓ ESP-NOW initialized");

  esp_now_register_recv_cb(onDataRecv);
  Serial.println("✓ Receiver callback registered");

  #if ENABLE_I2C_FORWARD
    Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
    Serial.print("✓ I2C initialized → Arduino at 0x");
    Serial.println(ARDUINO_I2C_ADDRESS, HEX);
  #else
    Serial.println("✓ I2C forwarding disabled");
  #endif

  Serial.println("\nWaiting for packets...\n");
}

// ========== MAIN LOOP ==========
void loop() {
  if (packetCount > 0 && (millis() - lastReceiveTime) > 5000) {
    Serial.println("WARNING: No packets in 5 seconds");
    delay(1000);
  }
  delay(100);
}
