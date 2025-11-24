/*
 * ESP-NOW Vibe Receiver
 *
 * Receives vibe state packets from the microphone ESP32 via ESP-NOW
 * Displays received data on Serial Monitor
 *
 * Upload this to your receiver ESP32 first to get its MAC address
 * Then use that MAC address in the sender's code
 *
 * What you can do with the received data:
 * - Control LEDs based on vibe state
 * - Drive servos/motors
 * - Update LCD/OLED displays
 * - Forward to Arduino via I2C
 * - Log to SD card
 */

#include <WiFi.h>
#include <esp_now.h>

// Vibe State Enum (must match sender)
enum VibeState {
  SILENT = 0,      // Very quiet, minimal activity
  AMBIENT = 1,     // Background noise, no distinct activity
  CONVERSATION = 2,// Speech detected (mids dominant)
  ACTIVE = 3,      // Loud/energetic but not speech-heavy
  PARTY = 4        // Very loud, high energy
};

// Data packet structure (must match sender)
struct VibePacket {
  uint8_t db_percent;      // 0-100: Current dB as % of adaptive range
  uint8_t bass_percent;    // 0-100: Bass energy as % of adaptive range
  uint8_t mids_percent;    // 0-100: Mids energy as % of adaptive range
  uint8_t highs_percent;   // 0-100: Highs energy as % of adaptive range
  uint8_t vibe_state;      // VibeState enum value
  int8_t db_delta;         // -100 to +100: dB change rate (trend)
} __attribute__((packed));

// Last received packet
VibePacket lastPacket;
unsigned long lastReceiveTime = 0;
unsigned long packetCount = 0;

// ESP-NOW receive callback (compatible with both old and new ESP-IDF versions)
void onDataRecv(const esp_now_recv_info_t *recv_info, const uint8_t *data, int data_len) {
  // Verify packet size
  if (data_len != sizeof(VibePacket)) {
    Serial.println("ERROR: Received packet size mismatch");
    return;
  }

  // Copy received data into our struct
  memcpy(&lastPacket, data, sizeof(VibePacket));
  lastReceiveTime = millis();
  packetCount++;

  // Print sender MAC address
  Serial.print("From: ");
  for (int i = 0; i < 6; i++) {
    Serial.printf("%02X", recv_info->src_addr[i]);
    if (i < 5) Serial.print(":");
  }
  Serial.print(" | Packet #");
  Serial.println(packetCount);

  // Print vibe state
  Serial.print("Vibe: ");
  Serial.print(vibeStateToString((VibeState)lastPacket.vibe_state));
  Serial.print(" (");
  Serial.print(lastPacket.vibe_state);
  Serial.println(")");

  // Print levels
  Serial.print("Levels: dB=");
  Serial.print(lastPacket.db_percent);
  Serial.print("% | Bass=");
  Serial.print(lastPacket.bass_percent);
  Serial.print("% | Mids=");
  Serial.print(lastPacket.mids_percent);
  Serial.print("% | Highs=");
  Serial.print(lastPacket.highs_percent);
  Serial.println("%");

  // Print trend
  Serial.print("Trend: ");
  if (lastPacket.db_delta > 10) {
    Serial.print("GETTING LOUDER (+");
    Serial.print(lastPacket.db_delta);
    Serial.println(")");
  } else if (lastPacket.db_delta < -10) {
    Serial.print("GETTING QUIETER (");
    Serial.print(lastPacket.db_delta);
    Serial.println(")");
  } else {
    Serial.print("STEADY (");
    Serial.print(lastPacket.db_delta);
    Serial.println(")");
  }

  // Visual bar for dB level
  Serial.print("Volume: [");
  int bars = map(lastPacket.db_percent, 0, 100, 0, 30);
  for (int i = 0; i < bars; i++) Serial.print("=");
  for (int i = bars; i < 30; i++) Serial.print(" ");
  Serial.println("]");
  Serial.println();

  // TODO: Add your custom code here
  // Example: Control LEDs based on vibe state
  // Example: Update OLED display
  // Example: Send to Arduino via I2C
}

void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10);

  Serial.println("\n======================================");
  Serial.println("ESP-NOW Vibe Receiver");
  Serial.println("======================================\n");

  // Initialize WiFi in station mode
  WiFi.mode(WIFI_STA);

  // Print MAC address (use this in sender code)
  Serial.println("✓ WiFi initialized");
  Serial.print("✓ MY MAC ADDRESS: ");
  Serial.println(WiFi.macAddress());
  Serial.println("\n>> Copy this MAC address to the sender's receiverMAC[] array <<\n");

  // Initialize ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("ERROR: ESP-NOW init failed");
    return;
  }
  Serial.println("✓ ESP-NOW initialized");

  // Register receive callback
  esp_now_register_recv_cb(onDataRecv);
  Serial.println("✓ Receiver callback registered");
  Serial.println("\nWaiting for vibe packets...\n");
}

void loop() {
  // Check for timeout (no packets received in 5 seconds)
  if (packetCount > 0 && (millis() - lastReceiveTime) > 5000) {
    Serial.println("⚠ WARNING: No packets received in 5 seconds");
    delay(1000);  // Print warning every second
  }

  // Everything happens in the callback, so loop can be empty
  // Add your custom code here if needed
  delay(100);
}

// Convert vibe state enum to readable string
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
