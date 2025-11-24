/*
 * SPH0645LM4H I2S MEMS Microphone + FFT Analysis
 * with Adaptive Auto-Calibration & Vibe Detection
 *
 * Standard Wiring:
 * BCLK  → GPIO14
 * WS    → GPIO15
 * DOUT  → GPIO32
 * SEL   → GND (Left channel)
 * 3V    → 3.3V
 * GND   → GND
 *
 * This sketch performs real-time audio analysis:
 * - Sound pressure level (dB SPL)
 * - FFT frequency analysis (512-point)
 * - Frequency band detection (Bass, Mids, Highs)
 * - Adaptive min/max tracking (auto-calibrates to room)
 * - Vibe state classification (SILENT/AMBIENT/CONVERSATION/ACTIVE/PARTY)
 * - Trend detection (getting louder/quieter)
 * - Data packet generation for ESP-NOW/I2C transmission
 *
 * Adaptive Calibration Features:
 * - Automatically learns room's noise floor (min) and typical loudness (max)
 * - Scales adjust dynamically over time
 * - Works in both quiet and loud environments
 * - Prevents scale collapse with minimum range enforcement
 *
 * Vibe Detection:
 * - Classifies room activity/occupancy for student spaces
 * - Two-axis detection: volume level + speech presence (mids analysis)
 * - States: SILENT (empty), AMBIENT (background), CONVERSATION (talking),
 *   ACTIVE (rowdy/energetic), PARTY (very loud)
 * - Outputs 6-byte packet: dB%, bass%, mids%, highs%, vibe_state, delta
 * - Ready for ESP-NOW transmission to other ESP32s
 * - Can be forwarded via I2C to Arduinos for LED/servo/display control
 *
 * Requires: arduinoFFT library
 * Open Serial Monitor (115200 baud) for stats
 * Open Serial Plotter for waveform visualization
 */

#include <driver/i2s.h>
#include <arduinoFFT.h>
#include <WiFi.h>
#include <esp_now.h>

// ESP-NOW Configuration
// Receiver ESP32 MAC address
uint8_t receiverMAC[] = {0xEC, 0xE3, 0x34, 0x1A, 0x7F, 0x38};

// I2S Configuration - Standard Pins
#define I2S_WS    15    // LRCL (Word Select)
#define I2S_SD    32    // DOUT (Serial Data)
#define I2S_SCK   14    // BCLK (Bit Clock)
#define I2S_PORT  I2S_NUM_0

// Audio Processing Configuration
#define SAMPLE_RATE       44100      // Standard audio rate (Hz)
#define SAMPLE_BITS       32         // SPH0645 outputs 18-bit in 32-bit frames
#define BUFFER_SIZE       512        // Samples per read
#define DISPLAY_INTERVAL  100        // Update display every 100ms

// Calibration (adjust based on your setup)
#define MIC_OFFSET_DB     94.0       // Reference SPL at 1kHz, 94dB = 1 Pa
#define MIC_SENSITIVITY   -26.0      // dBFS (SPH0645 typical: -26 dBFS)
#define MIC_REF_AMPL      pow(10, MIC_SENSITIVITY / 20.0)

// FFT Configuration
#define FFT_SIZE          512        // Must match BUFFER_SIZE
#define SAMPLING_FREQ     SAMPLE_RATE
#define FREQ_BIN_SIZE     (SAMPLING_FREQ / FFT_SIZE)  // ~86 Hz per bin

// Adaptive Calibration Configuration
#define ADAPTIVE_DECAY      0.9995    // Slow decay for max (0.9995 = ~2000 samples to halve)
#define ADAPTIVE_RISE       0.9995    // Slow rise for min (mirrors decay)
#define ADAPTIVE_ATTACK     1.05      // Fast response to increases
#define MIN_DB_RANGE        20.0      // Minimum dB range (prevents collapse)
#define MIN_BAND_RANGE      10000.0   // Minimum band energy range
#define DB_FLOOR            30.0      // Absolute minimum dB
#define DB_CEILING          100.0     // Absolute maximum dB
#define BAND_FLOOR          1000.0    // Absolute minimum band energy
#define BAND_CEILING        500000.0  // Absolute maximum band energy

// Global Variables
int32_t samples[BUFFER_SIZE];
size_t bytes_read;
unsigned long last_display = 0;

// Adaptive Calibration Tracking
float adaptive_db_min = 100.0;       // Start high, will adapt down
float adaptive_db_max = 30.0;        // Start low, will adapt up
float adaptive_band_min = 500000.0;  // Start high, will adapt down
float adaptive_band_max = 1000.0;    // Start low, will adapt up

// FFT Arrays
double vReal[FFT_SIZE];
double vImag[FFT_SIZE];
ArduinoFFT<double> FFT = ArduinoFFT<double>(vReal, vImag, FFT_SIZE, SAMPLING_FREQ);

// Vibe State Detection
enum VibeState {
  SILENT = 0,      // Very quiet, minimal activity
  AMBIENT = 1,     // Background noise, no distinct activity
  CONVERSATION = 2,// Speech detected (mids dominant)
  ACTIVE = 3,      // Loud/energetic but not speech-heavy
  PARTY = 4        // Very loud, high energy
};

// Data packet for transmission (ESP-NOW → I2C)
struct VibePacket {
  uint8_t db_percent;      // 0-100: Current dB as % of adaptive range
  uint8_t bass_percent;    // 0-100: Bass energy as % of adaptive range
  uint8_t mids_percent;    // 0-100: Mids energy as % of adaptive range
  uint8_t highs_percent;   // 0-100: Highs energy as % of adaptive range
  uint8_t vibe_state;      // VibeState enum value
  int8_t db_delta;         // -100 to +100: dB change rate (trend)
} __attribute__((packed));

// Delta tracking for trend detection
float last_db = 0;
unsigned long last_delta_time = 0;

// Vote-based state smoothing
#define STATE_HISTORY_SIZE 100  // Number of readings to average (at 100ms intervals = 10 seconds)
uint8_t state_history[STATE_HISTORY_SIZE];
int state_history_index = 0;
bool state_history_filled = false;

// Calculate smoothed vibe state using weighted average voting
VibeState getSmoothedVibeState() {
  // Count how many readings we actually have
  int valid_count = state_history_filled ? STATE_HISTORY_SIZE : state_history_index;

  if (valid_count == 0) {
    return AMBIENT;  // Default if no history yet
  }

  // Calculate weighted sum: sum all state values
  float sum = 0;
  for (int i = 0; i < valid_count; i++) {
    sum += state_history[i];
  }

  // Get the average and round to nearest state
  float average = sum / valid_count;
  int smoothed_state = round(average);

  // Clamp to valid range
  smoothed_state = constrain(smoothed_state, SILENT, PARTY);

  return (VibeState)smoothed_state;
}

// ESP-NOW send callback (compatible with both old and new ESP-IDF versions)
void onDataSent(const wifi_tx_info_t *mac_addr, esp_now_send_status_t status) {
  // Optional: uncomment for debugging transmission status
  // Serial.print("ESP-NOW Send Status: ");
  // Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Success" : "Fail");
}

void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10);

  Serial.println("\n======================================");
  Serial.println("SPH0645LM4H I2S Microphone + FFT Test");
  Serial.println("======================================\n");

  // Configure I2S for microphone input
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

  // Configure I2S pins
  i2s_pin_config_t pin_config = {
    .bck_io_num = I2S_SCK,
    .ws_io_num = I2S_WS,
    .data_out_num = I2S_PIN_NO_CHANGE,
    .data_in_num = I2S_SD
  };

  // Install and start I2S driver
  esp_err_t err = i2s_driver_install(I2S_PORT, &i2s_config, 0, NULL);
  if (err != ESP_OK) {
    Serial.printf("ERROR: I2S driver install failed: %d\n", err);
    while (1) delay(1000);
  }

  err = i2s_set_pin(I2S_PORT, &pin_config);
  if (err != ESP_OK) {
    Serial.printf("ERROR: I2S pin config failed: %d\n", err);
    while (1) delay(1000);
  }

  Serial.println("✓ I2S initialized successfully");
  Serial.println("✓ Sample Rate: 44100 Hz");
  Serial.println("✓ Bit Depth: 32-bit");
  Serial.println("✓ Channel: Left (Mono)");
  Serial.println("✓ FFT Size: 512 samples");
  Serial.print("✓ Frequency Resolution: ");
  Serial.print(FREQ_BIN_SIZE);
  Serial.println(" Hz per bin");
  Serial.println("✓ Adaptive Calibration: ENABLED");
  Serial.println("  - Auto-adjusts to room acoustics");
  Serial.println("  - Tracks min/max for dB and frequency bands");
  Serial.println("  - Scales adapt over time");
  Serial.println("✓ State Smoothing: ENABLED");
  Serial.print("  - Averaging over ");
  Serial.print(STATE_HISTORY_SIZE);
  Serial.println(" readings for stable vibe detection");
  Serial.println("  - Filters out brief pauses and fluctuations\n");
  Serial.println("Listening for audio...\n");

  // Initialize state history buffer
  for (int i = 0; i < STATE_HISTORY_SIZE; i++) {
    state_history[i] = AMBIENT;  // Start with neutral state
  }

  // Initialize WiFi and ESP-NOW
  WiFi.mode(WIFI_STA);
  Serial.print("✓ WiFi MAC Address: ");
  Serial.println(WiFi.macAddress());

  if (esp_now_init() != ESP_OK) {
    Serial.println("ERROR: ESP-NOW init failed");
    return;
  }
  Serial.println("✓ ESP-NOW initialized");

  // Register send callback
  esp_now_register_send_cb(onDataSent);

  // Add peer (receiver ESP32)
  esp_now_peer_info_t peerInfo;
  memcpy(peerInfo.peer_addr, receiverMAC, 6);
  peerInfo.channel = 0;  // Use current WiFi channel
  peerInfo.encrypt = false;  // No encryption for simplicity

  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("ERROR: Failed to add ESP-NOW peer");
    return;
  }
  Serial.println("✓ ESP-NOW peer added");
  Serial.print("  Receiver MAC: ");
  for (int i = 0; i < 6; i++) {
    Serial.printf("%02X", receiverMAC[i]);
    if (i < 5) Serial.print(":");
  }
  Serial.println("\n");

  delay(500); // Let I2S stabilize
}

void loop() {
  // Read I2S samples
  i2s_read(I2S_PORT, &samples, BUFFER_SIZE * sizeof(int32_t), &bytes_read, portMAX_DELAY);
  int samples_read = bytes_read / sizeof(int32_t);

  // Calculate RMS (Root Mean Square) for volume level
  float sum_squares = 0;

  for (int i = 0; i < samples_read; i++) {
    // SPH0645 outputs 18-bit data in 32-bit format, shift to get proper range
    int32_t sample = samples[i] >> 14;  // Convert to 18-bit signed
    sum_squares += (float)sample * sample;
  }

  // Calculate RMS amplitude
  float rms = sqrt(sum_squares / samples_read);

  // Convert RMS to dB SPL (Sound Pressure Level)
  float db = 0;
  if (rms > 0) {
    // Convert to dBFS (dB Full Scale)
    float dbfs = 20.0 * log10(rms / 131072.0); // 2^17 for 18-bit
    // Convert dBFS to dB SPL
    db = dbfs + MIC_OFFSET_DB - MIC_SENSITIVITY;
  } else {
    db = 0;  // Silence
  }

  // Perform FFT analysis
  // Populate FFT input arrays
  for (int i = 0; i < FFT_SIZE; i++) {
    vReal[i] = (double)(samples[i] >> 14);  // Convert to 18-bit signed
    vImag[i] = 0.0;  // Imaginary part is zero for real input
  }

  // Apply windowing function (reduces spectral leakage)
  FFT.windowing(FFTWindow::Hamming, FFTDirection::Forward);

  // Compute FFT
  FFT.compute(FFTDirection::Forward);

  // Compute magnitudes
  FFT.complexToMagnitude();

  // Analyze frequency bands
  // Note: Only use first half of FFT results (FFT_SIZE/2) due to Nyquist
  // Frequency bands (Hz per bin = ~86 Hz):
  // Bass: 20-250 Hz (bins 0-2)
  // Mids: 250-2000 Hz (bins 3-23)
  // Highs: 2000-8000 Hz (bins 24-93)

  float bass_energy = 0;
  float mids_energy = 0;
  float highs_energy = 0;

  // Sum energy in each frequency band
  for (int i = 1; i < FFT_SIZE / 2; i++) {  // Skip DC component (bin 0)
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

  // ===== ADAPTIVE CALIBRATION =====
  // Update adaptive dB range
  if (db > 0) {  // Only update with valid readings
    // Max tracking: decays slowly, jumps up quickly
    if (db > adaptive_db_max) {
      adaptive_db_max = db * ADAPTIVE_ATTACK;  // Fast rise
    } else {
      adaptive_db_max = adaptive_db_max * ADAPTIVE_DECAY;  // Slow decay
    }

    // Min tracking: rises slowly, drops quickly
    if (db < adaptive_db_min) {
      adaptive_db_min = db;  // Fast drop
    } else {
      adaptive_db_min = adaptive_db_min + (100.0 - adaptive_db_min) * (1.0 - ADAPTIVE_RISE);  // Slow rise
    }

    // Enforce minimum range to prevent collapse
    float db_range = adaptive_db_max - adaptive_db_min;
    if (db_range < MIN_DB_RANGE) {
      // Expand range around current value
      float center = (adaptive_db_max + adaptive_db_min) / 2.0;
      adaptive_db_max = center + MIN_DB_RANGE / 2.0;
      adaptive_db_min = center - MIN_DB_RANGE / 2.0;
    }

    // Apply absolute bounds
    adaptive_db_max = constrain(adaptive_db_max, DB_FLOOR, DB_CEILING);
    adaptive_db_min = constrain(adaptive_db_min, DB_FLOOR, DB_CEILING);
  }

  // Update adaptive band range
  float max_band = max(max(bass_energy, mids_energy), highs_energy);
  float min_band = min(min(bass_energy, mids_energy), highs_energy);

  if (max_band > 0) {
    // Max tracking: decays slowly, jumps up quickly
    if (max_band > adaptive_band_max) {
      adaptive_band_max = max_band * ADAPTIVE_ATTACK;  // Fast rise
    } else {
      adaptive_band_max = adaptive_band_max * ADAPTIVE_DECAY;  // Slow decay
    }

    // Min tracking: rises slowly, drops quickly
    if (min_band < adaptive_band_min && min_band > 0) {
      adaptive_band_min = min_band;  // Fast drop
    } else {
      adaptive_band_min = adaptive_band_min + (BAND_CEILING - adaptive_band_min) * (1.0 - ADAPTIVE_RISE);  // Slow rise
    }

    // Enforce minimum range
    float band_range = adaptive_band_max - adaptive_band_min;
    if (band_range < MIN_BAND_RANGE) {
      adaptive_band_max = adaptive_band_min + MIN_BAND_RANGE;
    }

    // Apply absolute bounds
    adaptive_band_max = constrain(adaptive_band_max, BAND_FLOOR, BAND_CEILING);
    adaptive_band_min = constrain(adaptive_band_min, BAND_FLOOR, BAND_CEILING);
  }

  // ===== VIBE PACKET CALCULATION =====
  VibePacket packet;

  // Calculate dB delta (rate of change per second)
  unsigned long now = millis();
  float time_diff = (now - last_delta_time) / 1000.0;  // Convert to seconds

  if (time_diff > 0 && last_delta_time > 0) {
    float db_change = db - last_db;
    float db_rate = db_change / time_diff;  // dB per second

    // Clamp to -100 to +100 range for int8_t
    packet.db_delta = constrain((int)(db_rate * 10), -100, 100);
  } else {
    packet.db_delta = 0;  // First reading or invalid time
  }

  last_db = db;
  last_delta_time = now;

  // Normalize values to 0-100 percentages
  float db_range = adaptive_db_max - adaptive_db_min;
  float band_range = adaptive_band_max - adaptive_band_min;

  if (db_range > 0) {
    packet.db_percent = constrain(
      (int)((db - adaptive_db_min) / db_range * 100.0),
      0, 100
    );
  } else {
    packet.db_percent = 0;
  }

  if (band_range > 0) {
    packet.bass_percent = constrain(
      (int)((bass_energy - adaptive_band_min) / band_range * 100.0),
      0, 100
    );
    packet.mids_percent = constrain(
      (int)((mids_energy - adaptive_band_min) / band_range * 100.0),
      0, 100
    );
    packet.highs_percent = constrain(
      (int)((highs_energy - adaptive_band_min) / band_range * 100.0),
      0, 100
    );
  } else {
    packet.bass_percent = 0;
    packet.mids_percent = 0;
    packet.highs_percent = 0;
  }

  // Detect instantaneous vibe state
  VibeState instant_state = detectVibe(db, bass_energy, mids_energy, highs_energy,
                                        adaptive_db_min, adaptive_db_max);

  // Store in history buffer (circular buffer)
  state_history[state_history_index] = instant_state;
  state_history_index++;
  if (state_history_index >= STATE_HISTORY_SIZE) {
    state_history_index = 0;
    state_history_filled = true;  // We've wrapped around, buffer is full
  }

  // Get smoothed state using weighted average voting
  VibeState smoothed_state = getSmoothedVibeState();
  packet.vibe_state = smoothed_state;

  // Send vibe packet via ESP-NOW (every loop iteration for real-time updates)
  esp_err_t result = esp_now_send(receiverMAC, (uint8_t *)&packet, sizeof(packet));
  // Uncomment for debugging:
  // if (result != ESP_OK) {
  //   Serial.println("ESP-NOW send failed");
  // }

  // Display results periodically
  if (now - last_display >= DISPLAY_INTERVAL) {
    last_display = now;

    // Print for Serial Monitor (detailed stats)
    Serial.print("Level: ");
    Serial.print(db, 1);
    Serial.print(" dB SPL | RMS: ");
    Serial.print(rms, 0);
    Serial.println();

    // Print frequency band energies
    Serial.print("FFT Bands → Bass: ");
    Serial.print(bass_energy, 0);
    Serial.print(" | Mids: ");
    Serial.print(mids_energy, 0);
    Serial.print(" | Highs: ");
    Serial.print(highs_energy, 0);
    Serial.println();

    // Visual bar graph for dB level (adaptive scale)
    printBar(db, adaptive_db_min, adaptive_db_max);

    // Visual representation of frequency bands (adaptive scale)
    Serial.print("Freq: [B:");
    printMiniBar(bass_energy, adaptive_band_min, adaptive_band_max);
    Serial.print(" M:");
    printMiniBar(mids_energy, adaptive_band_min, adaptive_band_max);
    Serial.print(" H:");
    printMiniBar(highs_energy, adaptive_band_min, adaptive_band_max);
    Serial.print("] (");
    Serial.print((int)(adaptive_band_min / 1000));
    Serial.print("-");
    Serial.print((int)(adaptive_band_max / 1000));
    Serial.println("k)");

    // Calibration status (helps monitor adaptive behavior)
    float db_range = adaptive_db_max - adaptive_db_min;
    float band_range = adaptive_band_max - adaptive_band_min;
    Serial.print("📊 Calibration: dB range=");
    Serial.print(db_range, 1);
    Serial.print(" dB | Band range=");
    Serial.print((int)(band_range / 1000));
    Serial.println("k");

    // Display vibe packet (ready for ESP-NOW transmission)
    Serial.println("--- VIBE PACKET (ESP-NOW Ready) ---");
    Serial.print("Vibe State: ");
    Serial.print(vibeStateToString((VibeState)packet.vibe_state));
    Serial.print(" (");
    Serial.print(packet.vibe_state);
    Serial.print(")");

    // Show instant state for debugging if different
    if (instant_state != smoothed_state) {
      Serial.print(" [instant: ");
      Serial.print(vibeStateToString(instant_state));
      Serial.print("]");
    }
    Serial.println();

    Serial.print("Levels (%)  → dB:");
    Serial.print(packet.db_percent);
    Serial.print(" B:");
    Serial.print(packet.bass_percent);
    Serial.print(" M:");
    Serial.print(packet.mids_percent);
    Serial.print(" H:");
    Serial.print(packet.highs_percent);
    Serial.println();

    Serial.print("Trend: ");
    if (packet.db_delta > 10) {
      Serial.print("↗ GETTING LOUDER (+");
      Serial.print(packet.db_delta);
      Serial.println(")");
    } else if (packet.db_delta < -10) {
      Serial.print("↘ GETTING QUIETER (");
      Serial.print(packet.db_delta);
      Serial.println(")");
    } else {
      Serial.print("→ STEADY (");
      Serial.print(packet.db_delta);
      Serial.println(")");
    }

    Serial.print("Packet size: ");
    Serial.print(sizeof(packet));
    Serial.println(" bytes");
    Serial.println();
  }

  // Output for Serial Plotter (raw waveform visualization)
  // Uncomment to enable plotter mode (disable Serial.print statements above)
  // Serial.println(samples[0] >> 14);
}

// Print visual bar graph of audio level (adaptive scale)
void printBar(float db, float db_min, float db_max) {
  Serial.print("[");
  int bars = map(constrain(db, db_min, db_max), db_min, db_max, 0, 40);
  for (int i = 0; i < bars; i++) {
    Serial.print("=");
  }
  for (int i = bars; i < 40; i++) {
    Serial.print(" ");
  }
  Serial.print("] (");
  Serial.print(db_min, 0);
  Serial.print("-");
  Serial.print(db_max, 0);
  Serial.println(" dB)");
}

// Print mini bar graph for frequency bands (adaptive scale)
void printMiniBar(float energy, float min_energy, float max_energy) {
  int bars = map(constrain(energy, min_energy, max_energy), min_energy, max_energy, 0, 8);
  for (int i = 0; i < bars; i++) {
    Serial.print("█");
  }
  for (int i = bars; i < 8; i++) {
    Serial.print("·");
  }
}

// Detect vibe state based on dB level and frequency distribution
// Uses two-axis approach: volume (relative_db) + speech detection (mids_ratio)
VibeState detectVibe(float db, float bass, float mids, float highs,
                     float db_min, float db_max) {
  // Calculate relative loudness (0.0 to 1.0+ in adaptive range)
  float relative_db = 0;
  if (db_max > db_min) {
    relative_db = (db - db_min) / (db_max - db_min);
  }

  // Calculate frequency distribution ratios
  float total_energy = bass + mids + highs;
  float mids_ratio = 0;

  if (total_energy > 0) {
    mids_ratio = mids / total_energy;
  }

  // Two-axis classification:
  // Axis 1: How loud is it? (relative_db)
  // Axis 2: Is it speech? (mids_ratio > 0.35)

  // SILENT: Empty/very quiet
  if (relative_db < 0.15 || db < 40) {
    return SILENT;
  }

  // AMBIENT: Light background noise
  if (relative_db < 0.40) {
    return AMBIENT;
  }

  // PARTY: Very loud
  if (relative_db > 0.75) {
    return PARTY;
  }

  // HIGH VOLUME: Check if it's speech or just noise/activity
  if (relative_db > 0.55) {
    if (mids_ratio > 0.35) {
      return CONVERSATION;  // People actively talking
    } else {
      return ACTIVE;  // Loud but not speech-heavy (rowdy, movement, etc.)
    }
  }

  // MODERATE VOLUME: Check if it's conversation or just background
  if (relative_db > 0.40) {
    if (mids_ratio > 0.35) {
      return CONVERSATION;  // Discussion/talking
    } else {
      return AMBIENT;  // Just background noise
    }
  }

  // Default: AMBIENT
  return AMBIENT;
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
