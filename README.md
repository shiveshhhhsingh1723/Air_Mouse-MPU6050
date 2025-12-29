


# 🖱️ ESP32 Air Mouse using MPU6050 (BLE HID)

A gesture-controlled wireless air mouse built using **ESP32 + MPU6050**, where hand motion controls the computer cursor over **Bluetooth Low Energy (BLE)** — no buttons, no touch, just motion.

This project demonstrates **sensor fusion**, **embedded systems**, **BLE HID implementation**, and **real-time motion control**.

---

## 🚀 Project Overview

The ESP32 reads **gyroscope data** from the MPU6050 IMU sensor and converts hand tilt into mouse cursor movement.  
The ESP32 behaves as a **Bluetooth HID Mouse**, allowing direct pairing with laptops and PCs without any drivers.

Once paired, moving your hand in air moves the cursor on the screen — similar to a VR controller or air remote.
[![Arduino](https://img.shields.io/badge/Arduino-IDE-00979D?logo=arduino)](https://www.arduino.cc/)
[![ESP32](https://img.shields.io/badge/ESP32-v3.3.5-E7352C?logo=espressif)](https://www.espressif.com/)
[![BLE](https://img.shields.io/badge/Bluetooth-BLE%20HID-0082FC?logo=bluetooth)](https://www.bluetooth.com/)
[![MPU6050](https://img.shields.io/badge/Sensor-MPU6050-FF6B6B)](https://invensense.tdk.com/)
[![License](https://img.shields.io/badge/license-MIT-yellow)](LICENSE)

---

## ✨ Key Features

- 🎯 **True Air Mouse** – cursor moves by hand motion
- 📡 **BLE HID Mouse** – works natively with Windows / Linux / macOS
- 🧠 **Gyroscope-based control** (no buttons used)
- 🔌 **Low power** Bluetooth Low Energy
- 🛠️ **ESP32 Core v3.3.5** compatible
- 📦 **No external BLE mouse libraries** required
- 🔄 **Real-time cursor updates** (~60 Hz)

---

## 🧰 Hardware Used

| Component      | Description                          |
|----------------|--------------------------------------|
| ESP32          | Main controller with BLE support     |
| MPU6050        | 6-axis IMU (Accelerometer + Gyroscope) |
| Jumper Wires   | For I²C communication                |
| Breadboard     | Prototyping                          |
| USB Cable      | Programming & power                  |

---

## 🔗 Wiring Connections

### MPU6050 → ESP32

| MPU6050 | ESP32   |
|---------|---------|
| VCC     | 3.3V    |
| GND     | GND     |
| SDA     | GPIO 21 |
| SCL     | GPIO 22 |

---

## 🧠 Working Principle

1. **MPU6050** measures angular velocity (gyro X & Y)
2. **ESP32** reads data via I²C
3. Gyro values are **mapped to mouse X/Y movement**
4. ESP32 sends **HID reports via BLE**
5. Laptop receives ESP32 as a **Bluetooth Mouse**
6. Cursor moves according to **hand tilt**

---

## 📡 Bluetooth Details

- **Device Name:** ESP32 Air Mouse
- **Profile:** Bluetooth HID (Human Interface Device)
- **Appearance:** Mouse
- **Protocol:** BLE (Low Energy)
- **Driver Required:** ❌ No

---

## 🖥️ Supported Platforms

- ✅ Windows 10 / 11
- ✅ Linux
- ✅ macOS
- ⚠️ Android / iOS (limited HID support)

---

## 🔧 Software & Libraries

- **Arduino IDE**
- **ESP32 Core v3.3.5**
- **NimBLE-Arduino**
- **Adafruit MPU6050 Library**
- **Adafruit Unified Sensor Library**

---

## 📥 Installation Steps

### 1️⃣ Install ESP32 Board Package

- Boards Manager → **ESP32 by Espressif**
- Version: **3.3.5**

### 2️⃣ Install Libraries

- NimBLE-Arduino
- Adafruit MPU6050
- Adafruit Unified Sensor

### 3️⃣ Upload Code

- Select correct COM port
- Upload sketch to ESP32

---

## 🔵 Pairing Instructions

1. Power ESP32
2. Open **Bluetooth Settings** on laptop
3. Search for devices
4. Connect to **ESP32 Air Mouse**
5. Cursor control starts automatically

---

## 🧪 Testing & Calibration

- Hold ESP32 **steady** after power-up
- Move hand **slowly** at first
- Sensitivity can be adjusted in code:

```cpp
int8_t xMove = g.gyro.y * 6;
int8_t yMove = g.gyro.x * -6;
```

- Increase multiplier for **faster movement**.


---

## 🧑‍💻 Learning Outcomes

- BLE HID implementation without external mouse libraries
- MPU6050 gyroscope handling
- Real-time embedded data streaming
- BLE services & characteristics
- Debugging ESP32 core compatibility issues

---

## 🏷️ Tags

`ESP32` `BLE` `HID` `Air Mouse` `MPU6050`  
`Embedded Systems` `IoT` `Gesture Control`

---

## 📜 License

This project is **open-source** and intended for **educational and research use**.

---

## 🤝 Contributing

Contributions are welcome! Feel free to:
- 🐛 Report bugs
- 💡 Suggest new features
- 🔧 Submit pull requests

---
