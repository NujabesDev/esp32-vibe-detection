# ESP32 + Multiple Arduino I2C Setup Guide

## System Architecture

```
ESP-NOW Transmitter → ESP32 Receiver (I2C Master) → Arduino 1 (0x08)
                                                   → Arduino 2 (0x09)
                                                   → Arduino 3 (0x0A)
```

**Data Flow:**
- ESP-NOW transmitter sends vibe data to ESP32
- ESP32 broadcasts vibe data to all Arduinos via I2C
- ESP32 polls each Arduino for button presses
- Each Arduino controls a servo based on vibe level

---

## I2C Wiring

### ESP32 (Master) Connections
- **SDA (GPIO 21)** → Connect to ALL Arduino SDA pins
- **SCL (GPIO 22)** → Connect to ALL Arduino SCL pins
- **GND** → Common ground with all Arduinos

### Arduino Connections (Each Arduino)
- **SDA (A4 on Uno/Nano)** → ESP32 SDA (GPIO 21)
- **SCL (A5 on Uno/Nano)** → ESP32 SCL (GPIO 22)
- **GND** → Common ground

**Important:** Use 4.7kΩ pull-up resistors on SDA and SCL lines (one pair total, not per device)

```
        4.7kΩ         4.7kΩ
         ┃             ┃
VCC ─────┼─────────────┼─────
         │             │
         │             │
    ─────┴─────────────┴───── SDA
         │             │
    ─────┼─────────────┼───── SCL
         │             │
       ESP32        Arduinos
```

---

## I2C Address Configuration

### Arduino 1 (Primary - with ultrasonic)
```cpp
#define MY_I2C_ADDRESS 0x08
#define ENABLE_ULTRASONIC true
```

### Arduino 2
```cpp
#define MY_I2C_ADDRESS 0x09
#define ENABLE_ULTRASONIC false
```

### Arduino 3
```cpp
#define MY_I2C_ADDRESS 0x0A
#define ENABLE_ULTRASONIC false
```

---

## Pin Assignments (Per Arduino)

| Component | Pin | Notes |
|-----------|-----|-------|
| Button 1 | 7 | INPUT_PULLUP |
| Button 2 | 6 | INPUT_PULLUP |
| Button 3 | 5 | INPUT_PULLUP |
| Button 4 | 4 | INPUT_PULLUP |
| Button 5 | 3 | INPUT_PULLUP |
| Servo | 10 | PWM capable |
| Ultrasonic Trig | 8 | Optional |
| Ultrasonic Echo | 9 | Optional |
| I2C SDA | A4 | Connect to ESP32 GPIO 21 |
| I2C SCL | A5 | Connect to ESP32 GPIO 22 |

---

## Upload Instructions

### 1. Upload ESP32 Code
1. Open `vibe_receiver.ino` in Arduino IDE
2. Select ESP32 board
3. Upload to ESP32
4. Note the MAC address printed in Serial Monitor

### 2. Upload Arduino Code (Repeat for each Arduino)
1. Open `arduino_slave.ino`
2. **CHANGE** `MY_I2C_ADDRESS` at the top:
   - Arduino 1: `0x08`
   - Arduino 2: `0x09`
   - Arduino 3: `0x0A`
3. Set `ENABLE_ULTRASONIC` to `true` on only ONE Arduino (if using sensor)
4. Upload to each Arduino
5. Open Serial Monitor to verify it's responding

---

## Testing

### Test 1: I2C Communication
1. Power on ESP32 and all Arduinos
2. Open ESP32 Serial Monitor (115200 baud)
3. Wait for vibe packets from transmitter
4. You should see:
   ```
   ✓ Arduino 0x08 OK
   ✓ Arduino 0x09 OK
   ✓ Arduino 0x0A OK
   ```

### Test 2: Button Input
1. Press any button on any Arduino
2. ESP32 Serial Monitor should show:
   ```
   Arduino 0x08 - Button 3 pressed
   ```
3. Arduino Serial Monitor should show:
   ```
   Button 3 pressed
     → Servo to 90°
   Sent button 3 to ESP32
   ```

### Test 3: Servo Control
1. Send vibe data from transmitter
2. Watch servos on all Arduinos move in sync
3. Servo angle = dB level mapped to 0-180°

---

## Troubleshooting

### "Arduino not responding" errors
- **Check wiring:** Verify SDA/SCL connections
- **Check power:** All devices need common ground
- **Check pull-ups:** Add 4.7kΩ resistors if missing
- **Check addresses:** Ensure each Arduino has unique address

### Buttons not registering
- **Check Serial Monitor** on Arduino (not ESP32)
- **Verify pin numbers** match your wiring
- **Test pullups:** Buttons should read HIGH when not pressed

### Servo not moving
- **Check power:** Servos need separate 5V supply for higher current
- **Check pin:** Verify servo is on pin 10
- **Check vibe data:** Open Arduino Serial Monitor to see if data arriving

### Garbage data
- **Check struct alignment:** Both ESP32 and Arduino must have same struct
- **Check wire length:** Keep I2C wires under 1 meter
- **Reduce speed:** Add `Wire.setClock(100000);` in ESP32 setup if needed

---

## Customization

### Change Servo Behavior
Edit `updateServoFromVibe()` in `arduino_slave.ino`:

```cpp
void updateServoFromVibe() {
  // Option 1: Use bass level instead of dB
  int targetAngle = map(vibeData.bass_percent, 0, 100, 0, 180);

  // Option 2: Different ranges per vibe state
  switch (vibeData.vibe_state) {
    case 0: targetAngle = 0; break;    // SILENT
    case 1: targetAngle = 45; break;   // AMBIENT
    case 2: targetAngle = 90; break;   // CONVERSATION
    case 3: targetAngle = 135; break;  // ACTIVE
    case 4: targetAngle = 180; break;  // PARTY
  }

  moveServoTo(targetAngle);
}
```

### Change Button Actions
Edit `handleButtonAction()` in `arduino_slave.ino` to customize what each button does.

---

## Performance Notes

- **I2C Speed:** Default 100kHz, can increase to 400kHz if needed
- **Polling Rate:** ESP32 requests button data every 100ms
- **Vibe Update:** Real-time when transmitter sends new packet
- **Max Cable Length:** 1 meter for reliable I2C communication
- **Max Devices:** Current setup handles 3 Arduinos, can expand to ~100 devices

---

## Files

- `vibe_receiver.ino` - ESP32 I2C Master (receives ESP-NOW, forwards to Arduinos)
- `arduino_slave.ino` - Arduino I2C Slave (receives vibe, controls servo, sends buttons)
- `vibe_transmitter.ino` - ESP-NOW transmitter (unchanged)

---

## Need Help?

Check Serial Monitor output at 115200 baud for detailed debugging info!
