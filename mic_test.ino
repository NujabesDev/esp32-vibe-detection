/*
 * SPH0645LM4H I2S MEMS Microphone + FFT Analysis
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
 * - Dominant frequency detection
 * - Peak detection
 *
 * Requires: arduinoFFT library
 * Open Serial Monitor (115200 baud) for stats
 * Open Serial Plotter for waveform visualization
 */

#include <driver/i2s.h>
#include <arduinoFFT.h>

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

// FFT Band Scaling (adjust these based on your room's typical levels)
// To calibrate: Watch the FFT Bands values in serial output
// - Quiet room: ~5k-20k per band
// - Normal speech/music: ~50k-150k per band
// - Loud music/party: ~200k-500k+ per band
// Set BAND_SCALE_MAX to your "loud but normal" level
#define BAND_SCALE_MAX    200000.0   // Max energy for full bar

// Global Variables
int32_t samples[BUFFER_SIZE];
size_t bytes_read;
unsigned long last_display = 0;
int32_t peak_max = 0;
int32_t peak_min = 0;

// FFT Arrays
double vReal[FFT_SIZE];
double vImag[FFT_SIZE];
ArduinoFFT<double> FFT = ArduinoFFT<double>(vReal, vImag, FFT_SIZE, SAMPLING_FREQ);

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
  Serial.println(" Hz per bin\n");
  Serial.println("Listening for audio...\n");

  delay(500); // Let I2S stabilize
}

void loop() {
  // Read I2S samples
  i2s_read(I2S_PORT, &samples, BUFFER_SIZE * sizeof(int32_t), &bytes_read, portMAX_DELAY);
  int samples_read = bytes_read / sizeof(int32_t);

  // Calculate RMS (Root Mean Square) for volume level
  float sum_squares = 0;
  int32_t current_max = INT32_MIN;
  int32_t current_min = INT32_MAX;

  for (int i = 0; i < samples_read; i++) {
    // SPH0645 outputs 18-bit data in 32-bit format, shift to get proper range
    int32_t sample = samples[i] >> 14;  // Convert to 18-bit signed

    sum_squares += (float)sample * sample;

    // Track peaks
    if (sample > current_max) current_max = sample;
    if (sample < current_min) current_min = sample;

    // Update global peaks
    if (sample > peak_max) peak_max = sample;
    if (sample < peak_min) peak_min = sample;
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

  // Find dominant frequency (peak)
  double peak_freq = FFT.majorPeak();

  // Display results periodically
  unsigned long now = millis();
  if (now - last_display >= DISPLAY_INTERVAL) {
    last_display = now;

    // Print for Serial Monitor (detailed stats)
    Serial.print("Level: ");
    Serial.print(db, 1);
    Serial.print(" dB SPL | RMS: ");
    Serial.print(rms, 0);
    Serial.print(" | Peak Freq: ");
    Serial.print(peak_freq, 0);
    Serial.println(" Hz");

    // Print frequency band energies
    Serial.print("FFT Bands → Bass: ");
    Serial.print(bass_energy, 0);
    Serial.print(" | Mids: ");
    Serial.print(mids_energy, 0);
    Serial.print(" | Highs: ");
    Serial.print(highs_energy, 0);
    Serial.println();

    // Visual bar graph for dB level
    printBar(db);

    // Visual representation of frequency bands (fixed absolute scale)
    Serial.print("Freq: [B:");
    printMiniBar(bass_energy, BAND_SCALE_MAX);
    Serial.print(" M:");
    printMiniBar(mids_energy, BAND_SCALE_MAX);
    Serial.print(" H:");
    printMiniBar(highs_energy, BAND_SCALE_MAX);
    Serial.print("] (max=");
    Serial.print((int)(BAND_SCALE_MAX / 1000));
    Serial.println("k)");
    Serial.println();
  }

  // Output for Serial Plotter (raw waveform visualization)
  // Uncomment to enable plotter mode (disable Serial.print statements above)
  // Serial.println(samples[0] >> 14);
}

// Print visual bar graph of audio level
void printBar(float db) {
  Serial.print("[");
  int bars = map(constrain(db, 30, 100), 30, 100, 0, 40);
  for (int i = 0; i < bars; i++) {
    Serial.print("=");
  }
  for (int i = bars; i < 40; i++) {
    Serial.print(" ");
  }
  Serial.println("]");
}

// Print mini bar graph for frequency bands
void printMiniBar(float energy, float max_energy) {
  int bars = map(constrain(energy, 0, max_energy), 0, max_energy, 0, 8);
  for (int i = 0; i < bars; i++) {
    Serial.print("█");
  }
  for (int i = bars; i < 8; i++) {
    Serial.print("·");
  }
}

// Test summary (call this from loop if you want periodic summaries)
void printSummary() {
  Serial.println("\n--- Microphone Test Summary ---");
  Serial.print("Peak Amplitude: ");
  Serial.print(peak_max);
  Serial.print(" / ");
  Serial.println(peak_min);
  Serial.print("Dynamic Range: ");
  Serial.print((peak_max - peak_min));
  Serial.println(" (18-bit max: 262144)");
  Serial.println("-------------------------------\n");
}
