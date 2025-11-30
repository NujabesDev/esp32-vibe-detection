# ESP32 Vibe Detection - Quick Setup

Wireless room activity detector: Microphone → ESP32 Transmitter → ESP32 Receiver → Arduino

## Setup Steps

### 1. Upload Receiver ESP32

```
1. Open vibe_receiver.ino in Arduino IDE
2. Select ESP32 board and port
3. Upload
4. Open Serial Monitor (115200 baud)
5. COPY THE MAC ADDRESS (format: XX:XX:XX:XX:XX:XX)
```

### 2. Upload Transmitter ESP32

```
1. Open vibe_transmitter.ino in Arduino IDE
2. Install library: Sketch → Include Library → Manage Libraries → "arduinoFFT" → Install
3. Find line 14: uint8_t receiverMAC[] = {...};
4. PASTE YOUR RECEIVER'S MAC ADDRESS from Step 1
   Example: {0x24, 0x6F, 0x28, 0xAB, 0xCD, 0xEF}
5. Select ESP32 board (the one with microphone wired)
6. Upload
7. Open Serial Monitor (115200 baud)
```

### 3. Arduino (Optional - if you have I2C wired)

```
1. Wire: ESP32 GPIO21(SDA)→Arduino A4, GPIO22(SCL)→A5, GND→GND
2. Open arduino_slave_example.ino
3. Upload to Arduino
4. Open Serial Monitor (115200 baud)
```

## Microphone Wiring (ESP32 Transmitter)

```
SPH0645 → ESP32
─────────────────
BCLK  → GPIO14
WS    → GPIO15
DOUT  → GPIO32
SEL   → GND
3V    → 3.3V
GND   → GND
```

## Testing

**Transmitter should show:**
- Audio levels in dB SPL
- Frequency bands (Bass/Mids/Highs)
- Vibe state: SILENT/AMBIENT/CONVERSATION/ACTIVE/PARTY
- Bar graphs

**Receiver should show:**
- Incoming packets from transmitter MAC
- Same vibe state as transmitter
- Trend indicators (↗ louder / ↘ quieter)

**Quick tests:**
- Stay quiet → SILENT
- Talk normally → CONVERSATION
- Clap/shout → ACTIVE or PARTY

## Troubleshooting

**"No packets received":**
- Check MAC address in transmitter exactly matches receiver
- Both ESP32s powered on
- Within WiFi range

**"I2C Error":**
- Check wiring: SDA→A4, SCL→A5, GND→GND
- Verify Arduino I2C address matches (default 0x08)

**Audio stuck at 0:**
- Check microphone wiring (DOUT → GPIO32)
- Verify microphone power (3.3V)
- Check I2S initialization messages

## Vibe States

| State | Description | dB Range |
|-------|-------------|----------|
| SILENT | Empty room, very quiet | < 45 dB |
| AMBIENT | Background noise, studying | 45-60 dB |
| CONVERSATION | People talking | 60-75 dB (high mids) |
| ACTIVE | Loud/rowdy activity | 75-85 dB |
| PARTY | Very loud, high energy | > 85 dB |

**Note:** CONVERSATION vs AMBIENT/ACTIVE is detected by mids-frequency ratio (speech detection).

## Adjusting Thresholds

Edit `vibe_transmitter.ino` lines 36-40 if your room is quieter/louder:
```cpp
#define DB_SILENT      45.0  // Adjust for your environment
#define DB_AMBIENT     60.0
#define DB_MODERATE    75.0
#define DB_PARTY       85.0
```
