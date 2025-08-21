# 🛩️ ESP32 Dual-Core Airplane Control System ✈️

[![ESP32](https://img.shields.io/badge/ESP32-PICO--D4-blue?style=for-the-badge&logo=espressif)](https://www.espressif.com/)
[![PlatformIO](https://img.shields.io/badge/PlatformIO-Core-orange?style=for-the-badge&logo=platformio)](https://platformio.org/)
[![License](https://img.shields.io/badge/License-MIT-green?style=for-the-badge)](LICENSE)

> 🚀 **Advanced dual-core flight control system built for ESP32, featuring real-time LoRa communication, OLED display, and precision servo control for RC aircraft.**

---

## 🌟 Features

### 🎯 **Core Capabilities**
- ⚡ **Dual-Core Architecture** - Dedicated cores for flight control and communication
- 📡 **LoRa Communication** - Long-range wireless control with packet validation
- 🖥️ **OLED Display** - Real-time flight data visualization
- 🎮 **PS5 Controller Support** - Wireless controller integration
- 🛡️ **Safety Systems** - Connection timeout and emergency procedures
- 🔧 **Advanced Trim Control** - Precise flight surface adjustments

### 🛩️ **Flight Control**
- ⚡ **Engine/Throttle Control** (0-100%)
- 🎯 **Rudder Control** (±30° freedom)
- ⬆️⬇️ **Elevator Control** (Full range)
- 🛩️ **Aileron Control** (Dual servo setup)
- 🪶 **Flap Control** (0-4 positions)
- 🛑 **Emergency Airbrake**

### 🎛️ **Flight Modes**
- 🎮 **Manual Mode** - Direct pilot control
- 📐 **Stability Mode** - Computer-assisted stabilization
- 🎢 **Acrobatic Mode** - High-performance maneuvers
- 🛬 **Landing Mode** - Automated landing assistance

---

## 🔧 Hardware Requirements

### 📟 **Main Controller**
- 🎯 **ESP32-PICO-D4** (Revision v1.1)
  - ⚡ Dual Core 240MHz
  - 📶 WiFi + 📱 Bluetooth
  - 💾 Embedded Flash
  - 🔮 40MHz Crystal

### 📡 **Communication**
- **LoRa Module** - Long range control
- **OLED Display** (SSD1306, I2C) - Status monitoring
- **PS5 Controller** (Bluetooth) - Primary input

---

## 🚀 Getting Started

### 📋 **Prerequisites**
```bash
# Install PlatformIO Core
pip install platformio

# Or use PlatformIO IDE extension in VS Code
```

### ⬇️ **Installation**
1. **Clone the repository**
   ```bash
   git clone https://github.com/Arsalan134/Airplane.git
   cd Airplane
   ```

2. **Build the project**
   ```bash
   pio build
   ```

3. **Upload to ESP32**
   ```bash
   pio upload
   ```

4. **Monitor serial output**
   ```bash
   pio device monitor
   ```

---

## 🏗️ **Project Structure**

```
📁 Airplane/
├── 📄 LICENSE                         # 📝 MIT License file
├── 📄 platformio.ini                  # 🔧 PlatformIO configuration
├── � README.md                       # 📚 This comprehensive documentation
├── 📄 README_DUAL_CORE.md             # 🏗️ Dual-core implementation details
├── 📂 include/                        # 📂 Header files directory
│   ├── 📄 Airplane.h                  # 🛩️ Aircraft class definition
│   ├── 📄 common.h                    # 🔗 Shared definitions and pin config
│   ├── 📄 Display.h                   # 📺 Display interface and prototypes
│   ├── 📄 images.h                    # 🖼️ OLED graphics and icons
│   ├── 📄 main.h                      # 🔧 Main header includes
│   └── 📄 SD-Card.h                   # 💾 Storage definitions
├── 📂 src/                           # � Source code directory
│   ├── 📄 main.cpp                    # �🚀 Main dual-core setup and control loop
│   ├── 📄 Airplane.cpp               # ✈️ Flight control logic and servo management
│   ├── 📄 Display.cpp                # �️ OLED display functions and UI
│   ├── 📄 Lora.cpp                   # � LoRa communication and protocol handling
│   └── 📄 SD-Card.cpp                # 💾 Data logging functionality
└── 📂 test/                          # 📂 Unit tests directory
    ├── � README                      # 📚 Test documentation
    ├── 📂 test_airplane/              # ✈️ Aircraft control tests
    │   └── 📄 test_airplane.cpp       # 🧪 Aircraft functionality unit tests
    ├── 📂 test_lora/                  # 📡 LoRa communication tests
    │   └── 📄 test_lora.cpp           # 🧪 LoRa protocol unit tests
    ├── � test_main/                  # � Main functionality tests
    │   ├── 📄 test_main.cpp           # 🧪 Main system integration tests
    │   └── 📄 test_main_simple.cpp    # 🧪 Simplified main function tests
    └── 📂 test_safety/                # �️ Safety system tests
        └── 📄 test_safety.cpp         # 🧪 Safety feature validation tests
```

### 📋 **Key Files Description**

#### � **Core System Files**
- **`main.cpp`**: Dual-core task implementation with FreeRTOS scheduling
- **`common.h`**: Shared definitions, pin configurations, and system constants
- **`main.h`**: Function declarations and system-wide includes

#### ✈️ **Flight Control System**
- **`Airplane.cpp/.h`**: Complete aircraft control logic, servo management, and flight algorithms
- **`Display.cpp/.h`**: OLED display management with real-time flight data visualization
- **`images.h`**: Graphics definitions, icons, and display assets

#### � **Communication System**
- **`Lora.cpp`**: LoRa communication protocol, packet handling, and error recovery
- **`SD-Card.cpp/.h`**: Data logging system for flight data recording

#### 🧪 **Testing Framework**
- **`test/`**: Comprehensive unit testing suite with dedicated test categories
  - **`test_airplane/`**: Aircraft-specific functionality tests
  - **`test_lora/`**: LoRa communication and protocol tests  
  - **`test_main/`**: Core system and integration tests
  - **`test_safety/`**: Critical safety system validation tests

#### 📚 **Documentation & Configuration**
- **`README.md`**: This comprehensive project documentation
- **`README_DUAL_CORE.md`**: Detailed dual-core architecture implementation guide
- **`platformio.ini`**: Build configuration with dual-core optimization flags
- **`LICENSE`**: MIT License terms and conditions

---

## 📡 **Communication Protocol**

The system uses a custom LoRa protocol with the following data structure:

### 📦 **Packet Format**
```
e[engine]a[aileron]r[rudder]l[elevator]t[trim]i[aileron_trim]f[flaps]z[reset_aileron]y[reset_elevator]b[airbrake]#[checksum]
```

### 🎛️ **Control Mapping**
| Command | Range | Description | Emoji |
|---------|-------|-------------|-------|
| `e` | 0-180 | Engine throttle | ⚡ |
| `a` | 0-180 | Aileron position (90=center) | 🛩️ |
| `r` | 0-180 | Rudder position (90=center) | 🎯 |
| `l` | 0-180 | Elevator position (90=center) | ⬆️⬇️ |
| `t` | -1,0,1 | Elevator trim adjustment | 🔧 |
| `i` | -1,0,1 | Aileron trim adjustment | 🔧 |
| `f` | 0-4 | Flap position | 🪶 |
| `z` | 0-1 | Reset aileron trim | 🔄 |
| `y` | 0-1 | Reset elevator trim | 🔄 |
| `b` | 0-1 | Airbrake activation | 🛑 |

### 🔐 **Safety Features**
- ✅ **XOR Checksum** - Packet integrity verification
- ⏱️ **Connection Timeout** - 2 second failsafe
- 🚨 **Emergency Procedures** - Auto-safe mode on signal loss
- 🔒 **Packet Validation** - Malformed packet rejection

---

## 🖥️ **Display Interface**

The OLED display shows real-time flight data:

### 📊 **Status Information**
- 🎯 Aileron position
- ⬆️⬇️ Elevator position  
- 🎯 Rudder position
- 🔧 Trim values (elevator & aileron)
- 📶 Signal strength (RSSI)
- ⏱️ Last packet time
- ⚡ Engine power percentage
- 🪶 Flap position
- 👨‍✈️ Pilot identification

---

## ⚡ **Dual-Core Architecture**

### 🎯 **Core 1 - Flight Control** (High Priority)
- 🛩️ Real-time servo control
- 🚨 Emergency safety systems  
- 📊 Performance monitoring
- 🔄 100Hz update rate

### 📡 **Core 0 - Communication** (Lower Priority)
- 📻 LoRa packet handling
- 🖥️ Display updates
- 📱 Bluetooth management
- 🔄 50Hz update rate

---

## 🛡️ **Safety Systems**

### 🚨 **Emergency Procedures**
1. **Connection Loss** - Automatic safe mode activation
2. **Invalid Packets** - Ignore corrupted data
3. **Hardware Timeout** - Servo centering
4. **Memory Protection** - Mutex-based data sharing

### 🔧 **Failsafe Defaults**
- ⚡ Engine: 0% (Motor off)
- 🛩️ All control surfaces: Center position (90°)
- 📐 Flight mode: Stability mode
- 🛑 Airbrakes: Deactivated

---

## 📈 **Performance Monitoring**

The system provides real-time performance metrics:

- 🔄 **Update Frequencies** - Per-core execution rates
- 💾 **Memory Usage** - Heap and stack monitoring  
- ⏱️ **Task Timing** - Microsecond precision logging
- 📊 **Communication Stats** - Packet success rates

---

## 🤝 **Contributing**

We welcome contributions! Here's how you can help:

1. 🍴 **Fork** the repository
2. 🌟 **Create** a feature branch (`git checkout -b feature/amazing-feature`)
3. 💾 **Commit** your changes (`git commit -m '✨ Add amazing feature'`)
4. 📤 **Push** to the branch (`git push origin feature/amazing-feature`)
5. 🔃 **Open** a Pull Request

### 🎯 **Areas for Contribution**
- 📱 Mobile app development
- 🧪 Unit testing
- 📚 Documentation improvements
- 🔧 Hardware integration
- 🛩️ Flight algorithms

---

## 📝 **License**

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

---

## 👨‍💻 **Author**

**🛩️ Arsalan Iravani**
- 📧 Email: [airavani2018@gmail.com](mailto:your-email@example.com)
- 🐙 GitHub: [@Arsalan134](https://github.com/Arsalan134)

---

## 🙏 **Acknowledgments**

- 🚀 **ESP32 Community** - For excellent hardware support
- 📡 **LoRa Alliance** - For long-range communication standards  
- 🎮 **Gaming Community** - For PS5 controller integration inspiration
- ✈️ **RC Aviation Community** - For flight control insights

---

## 📊 **Project Stats**

- 🗓️ **Started**: 2024
- 💻 **Language**: C++ (Arduino Framework)
- 🎯 **Target**: ESP32-PICO-D4
- 📦 **Dependencies**: PlatformIO, Arduino, LoRa
- 🏷️ **Version**: 1.0.0

---

<div align="center">

### 🛩️ **Ready for Takeoff!** ✈️

*Built with ❤️ for the RC aviation community*

**⭐ Star this repository if you found it helpful!**

</div>
