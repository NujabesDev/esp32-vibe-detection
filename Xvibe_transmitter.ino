/*
 * ESP32 Vibe Transmitter - Microphone + FFT + ESP-NOW
 * Wiring: BCLK→GPIO14, WS→GPIO15, DOUT→GPIO32, SEL→GND, 3V→3.3V, GND→GND
 * UPDATE LINE 14 WITH YOUR RECEIVER'S MAC ADDRESS!
 */

#include <driver/i2s.h>
#include <arduinoFFT.h>
#include <WiFi.h>
#include <esp_now.h>

// ========== CONFIGURATION ==========
// UPDATE THIS WITH YOUR RECEIVER'S MAC ADDRESS!
uint8_t receiverMAC[] = {0x24, 0x6F, 0x28, 0x2D, 0x18, 0xD8};

// I2S Pins
#define I2S_WS    15
#define I2S_SD    32
#define I2S_SCK   14
#define I2S_PORT  I2S_NUM_0

// Audio Settings
#define SAMPLE_RATE       44100
#define BUFFER_SIZE       512
#define DISPLAY_INTERVAL  100

// Microphone Calibration
#define MIC_OFFSET_DB     94.0
#define MIC_SENSITIVITY   -26.0

// FFT Settings
#define FFT_SIZE          512
#define SAMPLING_FREQ     SAMPLE_RATE
#define FREQ_BIN_SIZE     (SAMPLING_FREQ / FFT_SIZE)

// Fixed dB Thresholds (adjust for your room)
#define DB_SILENT      45.0
#define DB_AMBIENT     60.0
#define DB_MODERATE    75.0
#define DB_PARTY       85.0
#define DB_MAX         100.0

// Frequency Band Normalization
#define BAND_MAX_INITIAL  10000.0
#define BAND_DECAY        0.999    // Slow decay for max tracking

// Vibe Detection
#define SPEECH_MIDS_RATIO  0.35    // Mids ratio above this = speech

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
int32_t samples[BUFFER_SIZE];
size_t bytes_read;
unsigned long last_display = 0;

double vReal[FFT_SIZE];
double vImag[FFT_SIZE];
ArduinoFFT<double> FFT = ArduinoFFT<double>(vReal, vImag, FFT_SIZE, SAMPLING_FREQ);

float last_db = 0;
unsigned long last_delta_time = 0;

// Simple smoothing
float state_smooth = 1.0;  // Running average of vibe state (0-4 range)
float band_max = BAND_MAX_INITIAL;  // Running max for band normalization

// ========== FUNCTIONS ==========
void onDataSent(const wifi_tx_info_t *mac_addr, esp_now_send_status_t status) {}

VibeState detectVibe(float db, float mids_ratio) {
  VibeState instant_state;

  // Simple threshold-based detection
  if (db < DB_SILENT) {
    instant_state = SILENT;
  } else if (db < DB_AMBIENT) {
    instant_state = AMBIENT;
  } else if (db < DB_MODERATE) {
    instant_state = (mids_ratio > SPEECH_MIDS_RATIO) ? CONVERSATION : AMBIENT;
  } else if (db < DB_PARTY) {
    instant_state = (mids_ratio > SPEECH_MIDS_RATIO) ? CONVERSATION : ACTIVE;
  } else {
    instant_state = PARTY;
  }

  // Exponential smoothing (replaces 100-sample buffer)
  state_smooth = state_smooth * 0.85 + instant_state * 0.15;

  return (VibeState)round(constrain(state_smooth, 0, 4));
}

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

void printBar(float db) {
  Serial.print("[");
  int bars = map(constrain(db, 0, DB_MAX), 0, DB_MAX, 0, 40);
  for (int i = 0; i < bars; i++) Serial.print("=");
  for (int i = bars; i < 40; i++) Serial.print(" ");
  Serial.print("] ");
  Serial.print(db, 1);
  Serial.println(" dB");
}

// ========== SETUP ==========
void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10);

  Serial.println("\n=== ESP32 Vibe Transmitter ===\n");

  // Configure I2S
  i2s_config_t i2s_config = {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX),
    .sample_rate = SAMPLE_RATE,
    .bits_per_sample = I2S_BITS_PER_SAMPLE_32BIT,
    .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
    .communication_format = I2S_COMM_FORMAT_I2S,
    .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
    .dma_buf_count = 4,
    .dma_buf_len = BUFFER_SIZE,
    .use_apll = false,
    .tx_desc_auto_clear = false,
    .fixed_mclk = 0
  };

  i2s_pin_config_t pin_config = {
    .bck_io_num = I2S_SCK,
    .ws_io_num = I2S_WS,
    .data_out_num = I2S_PIN_NO_CHANGE,
    .data_in_num = I2S_SD
  };

  if (i2s_driver_install(I2S_PORT, &i2s_config, 0, NULL) != ESP_OK) {
    Serial.println("ERROR: I2S driver install failed");
    while (1) delay(1000);
  }

  if (i2s_set_pin(I2S_PORT, &pin_config) != ESP_OK) {
    Serial.println("ERROR: I2S pin config failed");
    while (1) delay(1000);
  }

  Serial.println("✓ I2S initialized");

  // Initialize WiFi and ESP-NOW
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  delay(100);
  Serial.print("✓ MAC Address: ");
  Serial.println(WiFi.macAddress());

  if (esp_now_init() != ESP_OK) {
    Serial.println("ERROR: ESP-NOW init failed");
    return;
  }
  Serial.println("✓ ESP-NOW initialized");

  esp_now_register_send_cb(onDataSent);

  esp_now_peer_info_t peerInfo;
  memset(&peerInfo, 0, sizeof(peerInfo));
  memcpy(peerInfo.peer_addr, receiverMAC, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;
  peerInfo.ifidx = WIFI_IF_STA;

  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("ERROR: Failed to add peer");
    return;
  }
  Serial.println("✓ Peer added\n");

  delay(500);
}

// ========== MAIN LOOP ==========
void loop() {
  // Read I2S samples
  i2s_read(I2S_PORT, &samples, BUFFER_SIZE * sizeof(int32_t), &bytes_read, portMAX_DELAY);
  int samples_read = bytes_read / sizeof(int32_t);

  // Calculate RMS
  float sum_squares = 0;
  for (int i = 0; i < samples_read; i++) {
    int32_t sample = samples[i] >> 14;
    sum_squares += (float)sample * sample;
  }
  float rms = sqrt(sum_squares / samples_read);

  // Convert to dB SPL
  float db = 0;
  if (rms > 0) {
    float dbfs = 20.0 * log10(rms / 131072.0);
    db = dbfs + MIC_OFFSET_DB - MIC_SENSITIVITY;
  }

  // FFT analysis
  for (int i = 0; i < FFT_SIZE; i++) {
    vReal[i] = (double)(samples[i] >> 14);
    vImag[i] = 0.0;
  }

  FFT.windowing(FFTWindow::Hamming, FFTDirection::Forward);
  FFT.compute(FFTDirection::Forward);
  FFT.complexToMagnitude();

  // Sum frequency bands
  float bass_energy = 0;   // 20-250 Hz
  float mids_energy = 0;   // 250-2000 Hz
  float highs_energy = 0;  // 2000-8000 Hz

  for (int i = 1; i < FFT_SIZE / 2; i++) {
    float magnitude = vReal[i];
    float freq = i * FREQ_BIN_SIZE;

    if (freq >= 20 && freq < 250) {
      bass_energy += magnitude;
    } else if (freq >= 250 && freq < 2000) {
      mids_energy += magnitude;
    } else if (freq >= 2000 && freq < 8000) {
      highs_energy += magnitude;
    }
  }

  // Update running max for normalization (with slow decay)
  float max_band = max(max(bass_energy, mids_energy), highs_energy);
  if (max_band > band_max) {
    band_max = max_band;
  } else {
    band_max *= BAND_DECAY;  // Slow decay
  }

  // Build packet
  VibePacket packet;
  unsigned long now = millis();

  // Calculate dB delta (rate of change)
  float time_diff = (now - last_delta_time) / 1000.0;
  if (time_diff > 0 && last_delta_time > 0) {
    float db_change = db - last_db;
    float db_rate = db_change / time_diff;
    packet.db_delta = constrain((int)(db_rate * 10), -100, 100);
  } else {
    packet.db_delta = 0;
  }
  last_db = db;
  last_delta_time = now;

  // Normalize dB to percentage (0-100)
  packet.db_percent = constrain(map(db, 0, DB_MAX, 0, 100), 0, 100);

  // Normalize frequency bands to percentage (0-100)
  if (band_max > 0) {
    packet.bass_percent = constrain((int)(bass_energy / band_max * 100.0), 0, 100);
    packet.mids_percent = constrain((int)(mids_energy / band_max * 100.0), 0, 100);
    packet.highs_percent = constrain((int)(highs_energy / band_max * 100.0), 0, 100);
  } else {
    packet.bass_percent = 0;
    packet.mids_percent = 0;
    packet.highs_percent = 0;
  }

  // Detect vibe state
  float total_energy = bass_energy + mids_energy + highs_energy;
  float mids_ratio = (total_energy > 0) ? (mids_energy / total_energy) : 0;

  packet.vibe_state = detectVibe(db, mids_ratio);

  // Send packet
  esp_now_send(receiverMAC, (uint8_t *)&packet, sizeof(packet));

  // Display (commented out to reduce serial monitor spam)
  // if (now - last_display >= DISPLAY_INTERVAL) {
  //   last_display = now;

  //   Serial.print("dB: ");
  //   Serial.print(db, 1);
  //   Serial.print(" | B:");
  //   Serial.print(bass_energy, 0);
  //   Serial.print(" M:");
  //   Serial.print(mids_energy, 0);
  //   Serial.print(" H:");
  //   Serial.print(highs_energy, 0);
  //   Serial.println();

  //   printBar(db);

  //   Serial.print("Vibe: ");
  //   Serial.print(vibeStateToString((VibeState)packet.vibe_state));
  //   Serial.print(" | Packet: dB=");
  //   Serial.print(packet.db_percent);
  //   Serial.print("% B=");
  //   Serial.print(packet.bass_percent);
  //   Serial.print(" M=");
  //   Serial.print(packet.mids_percent);
  //   Serial.print(" H=");
  //   Serial.print(packet.highs_percent);
  //   Serial.print(" Δ=");
  //   Serial.println(packet.db_delta);
  //   Serial.println();
  // }
}
