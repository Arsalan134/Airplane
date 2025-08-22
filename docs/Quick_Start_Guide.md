# Master-Slave Flight Control System - Quick Start Guide

## 🚀 Getting Started

### Step 1: Upload Code to Arduino Nano 33 BLE Sense
1. Open `nano_33_ble_servo_controller.ino` in Arduino IDE
2. Select board: "Arduino Nano 33 BLE" 
3. Install required libraries:
   - Arduino_LSM9DS1 (IMU)
   - Arduino_LPS22HB (Pressure)
   - Arduino_HTS221 (Temperature)
4. Upload the code

### Step 2: Modify Your TTGO Project  
1. Add the new files to your project:
   - `FlightProtocol.h`
   - `AirplaneMaster.h` 
   - `AirplaneMaster.cpp`
2. Replace your main.cpp with `main_master_slave.cpp`
3. Update your platformio.ini if needed

### Step 3: Wire the System
Follow the wiring diagram in `Wiring_Diagram.md`:
- Connect I2C between boards (SDA/SCL + GND)
- Connect servos to Arduino Nano PWM pins
- **Important**: Use separate 5V power supply for servos!

### Step 4: Test Communication
1. Power both boards
2. Check serial monitors:
   - TTGO should show "Slave communication established"
   - Arduino should show "Servo controller ready"

### Step 5: Test Servos
1. Send test commands via LoRa to your TTGO
2. Example command: `CMD:E:50,R:90,EL:85,RU:95`
3. Watch servos move smoothly to positions

---

## 🎮 Control Commands

### Basic Control:
- `CMD:E:0-180` - Engine throttle (0=off, 180=full)
- `CMD:R:0-180` - Roll/Ailerons (90=neutral)  
- `CMD:EL:0-180` - Elevators (90=neutral)
- `CMD:RU:0-180` - Rudder (90=neutral)

### Combined Commands:
- `CMD:E:50,R:90,EL:85,RU:95` - Set all controls at once

### Settings:
- `SET:MODE:MANUAL` - Set flight mode
- `SET:TRIM:EL:+5` - Elevator trim adjustment

---

## 📊 Telemetry Data

The system automatically sends telemetry every second:
```
TEL:R:-2.1,P:1.3,Y:87.5,A:125.3,T:23.2,S:OK
```
- R: Roll angle (degrees)
- P: Pitch angle (degrees)  
- Y: Yaw/heading (degrees)
- A: Altitude (meters)
- T: Temperature (°C)
- S: System status (OK/ERR)

---

## 🔧 Configuration

### Arduino Nano 33 BLE (Servo Controller):
```cpp
#define I2C_ADDRESS 0x08           // I2C slave address
#define ENGINE_SERVO_PIN 3         // PWM pins for servos
#define ROLL_LEFT_SERVO_PIN 5
#define ELEVATOR_LEFT_SERVO_PIN 6
#define ELEVATOR_RIGHT_SERVO_PIN 9
#define RUDDER_SERVO_PIN 10
```

### TTGO T3 (Master):
```cpp
#define SERVO_CONTROLLER_ADDRESS 0x08  // Must match slave address
```

---

## 🚨 Safety Features

1. **Connection Timeout**: If no commands received for 2 seconds, servos return to safe positions
2. **Checksum Validation**: All I2C packets have checksums to prevent corruption
3. **Smooth Movement**: Servos move gradually to prevent damage
4. **Health Monitoring**: Continuous monitoring of I2C communication and sensors
5. **Emergency Shutdown**: Master can send emergency stop command

---

## 🐛 Troubleshooting

### "Slave not responding":
- Check I2C wiring (SDA, SCL, GND)
- Verify both boards have common ground
- Check I2C address matches (0x08)

### "Servos not moving":
- Check servo power supply (5V, adequate current)
- Verify servo signal wires connected to correct pins
- Check serial monitor for error messages

### "IMU data not updating":
- Arduino Nano 33 BLE has built-in sensors, no wiring needed
- Check if IMU initialization succeeded in serial monitor

### "Communication errors":
- Check for electromagnetic interference near I2C lines
- Try lower I2C speed (50kHz instead of 100kHz)
- Ensure stable power supply to both boards

---

## 🎯 Next Steps

### Advanced Features to Add:
1. **PID Control**: Implement PID loops for stability
2. **Flight Modes**: Auto-level, altitude hold, return-to-home
3. **Data Logging**: Log flight data to SD card
4. **Bluetooth Telemetry**: Use Nano's BLE for phone app
5. **Failsafe**: GPS return-to-home if connection lost

### Sensor Fusion:
The Nano 33 BLE has excellent sensors - you can implement:
- Complementary filter for better attitude estimation
- Barometric altitude hold
- Magnetic compass heading
- Temperature-compensated altitude

---

## 📞 Support

If you need help:
1. Check serial monitor outputs from both boards
2. Verify all connections match the wiring diagram  
3. Test each component separately before integration
4. Monitor I2C communication with logic analyzer if available

Your master-slave system is now ready for flight! 🛩️
