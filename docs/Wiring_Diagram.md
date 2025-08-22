# Master-Slave Flight Control System Wiring

## TTGO T3 v1.6.1 (Master) Connections:

### Power:
- VCC → 3.3V or USB power
- GND → Common ground

### I2C (Communication with Servo Slave):
- Pin 21 (SDA) → Arduino Nano 33 BLE Pin A4/18 (SDA)
- Pin 22 (SCL) → Arduino Nano 33 BLE Pin A5/19 (SCL)

### LoRa (Built-in):
- Pin 27 → MOSI (internal)
- Pin 19 → MISO (internal)
- Pin 5 → SCK (internal)
- Pin 18 → CS (internal)
- Pin 26 → DIO0 (internal)
- Pin 23 → RST (internal)

### SD Card (Built-in):
- Pin 13 → CS (internal)
- Pin 15 → MOSI (internal)
- Pin 2 → MISO (internal)
- Pin 14 → SCK (internal)

### OLED Display (Built-in):
- Pin 21 → SDA (shared with I2C)
- Pin 22 → SCL (shared with I2C)

---

## Arduino Nano 33 BLE Sense (Servo Slave) Connections:

### Power:
- 3.3V or VIN → From TTGO or separate supply
- GND → Common ground with TTGO

### I2C (Communication with Master):
- A4/Pin 18 (SDA) → TTGO Pin 21 (SDA)
- A5/Pin 19 (SCL) → TTGO Pin 22 (SCL)

### Servos (5 servos total):
- Pin D3 (PWM) → Engine Servo Signal (Orange/Yellow wire)
- Pin D5 (PWM) → Roll Left Servo Signal
- Pin D6 (PWM) → Elevator Left Servo Signal  
- Pin D9 (PWM) → Elevator Right Servo Signal
- Pin D10 (PWM) → Rudder Servo Signal

### Servo Power Distribution:
- All servo RED wires → 5V power supply (NOT from Arduino!)
- All servo BLACK/BROWN wires → Common ground
- Servo signals connect to Arduino PWM pins as above

### Built-in Sensors (no wiring needed):
- LSM9DS1 IMU (accelerometer, gyroscope, magnetometer)
- LPS22HB pressure sensor
- HTS221 temperature/humidity sensor

---

## Power Supply Recommendations:

### For Servos:
- Use a separate 5V power supply rated for at least 2A per servo
- Common supplies: 5V 10A switching power supply
- Connect servo power supply ground to Arduino ground (common ground)

### For Electronics:
- TTGO can be powered via USB or 3.3V
- Arduino Nano 33 BLE can be powered from TTGO 3.3V or separate supply

---

## Servo Wiring Standard:
- RED/ORANGE: +5V power
- BLACK/BROWN: Ground
- YELLOW/WHITE/ORANGE: Signal (goes to Arduino PWM pin)

---

## Safety Notes:
1. ⚠️ **NEVER power servos from Arduino pins** - use separate power supply
2. 🔧 Always connect grounds together (common ground)
3. 🔌 Double-check I2C wiring (SDA to SDA, SCL to SCL)
4. 🔋 Ensure adequate power supply current for all servos
5. 📡 Test I2C communication before connecting servos
6. 🛡️ Add fuses on power lines for safety
