# Dum-E: 4-DOF Robotic Arm & Vision Dashboard

[![Python](https://img.shields.io/badge/Python-3.9%20%7C%203.10%20%7C%203.11-blue.svg)](https://www.python.org/)
[![ESP32](https://img.shields.io/badge/ESP32-Arduino-orange.svg)](https://www.espressif.com/)
[![OpenCV](https://img.shields.io/badge/OpenCV-Hough%20Circles-red.svg)](https://opencv.org/)
[![License: MIT](https://img.shields.io/badge/License-MIT-green.svg)](LICENSE)

An end-to-end autonomous robotic arm pick-and-place system featuring:
- **4-DOF Robotic Arm Control** (MG90S servos for Base, Shoulder, Elbow, Wrist + Electromagnet relay).
- **ESP32 Asynchronous REST API** running over Wi-Fi.
- **Computer Vision (OpenCV)**: Circular coin detection and rectangular colored sorting bin detection (Green, Red, Black, Blue, Yellow).
- **3D Inverse Kinematics (IK)** with interactive tap-to-set base reference origin.
- **Dum-E Web Dashboard**: Modern, high-contrast dashboard accessible from any desktop or mobile browser.
- **Hardware Simulator / Mock ESP32**: Full offline testing support with zero physical hardware connected.

---

## ⚡ Quick Start: How to Install on Another PC

### Method A: 1-Click Automated Setup (Recommended)

#### On Windows:
1. Clone or download the repository to the new PC:
   ```cmd
   git clone https://github.com/shashankbm666/Dum-E.git
   cd Dum-E
   ```
2. Double-click **`setup.bat`**.  
   *(This automatically creates `.venv` and installs all dependencies)*.
3. Double-click **`run.bat`**.  
   *(This starts the Vision Server + Hardware Simulator and opens your web browser to `http://localhost:8000/`)*.

#### On Linux / macOS / Raspberry Pi:
1. Open terminal and clone the repository:
   ```bash
   git clone https://github.com/shashankbm666/Dum-E.git
   cd Dum-E
   ```
2. Make scripts executable and run setup:
   ```bash
   chmod +x setup.sh run.sh
   ./setup.sh
   ```
3. Launch the application:
   ```bash
   ./run.sh
   ```

---

### Method B: Manual Installation (Command Line)

If you prefer installing manually using the terminal:

1. **Verify Python Installation:**
   Make sure Python 3.9, 3.10, or 3.11 is installed:
   ```bash
   python --version
   ```

2. **Clone the Repository:**
   ```bash
   git clone https://github.com/shashankbm666/Dum-E.git
   cd Dum-E
   ```

3. **Create & Activate a Virtual Environment:**
   - **Windows:**
     ```cmd
     python -m venv .venv
     .venv\Scripts\activate
     ```
   - **Linux / macOS:**
     ```bash
     python3 -m venv .venv
     source .venv/bin/activate
     ```

4. **Install Dependencies:**
   ```bash
   pip install -r requirements.txt
   ```

5. **Start the Command Center:**
   ```bash
   python run_server.py
   ```
   Open your browser to: **`http://localhost:8000/`**

---

## 📱 Controlling from a Mobile Phone / Tablet

You can control the robotic arm and use your smartphone's camera to snap photos of the workbench:

1. Ensure your phone and the computer are connected to the **same Wi-Fi network**.
2. Run `python run_server.py` on your computer. Look for the printed mobile URL:
   ```
   [DESKTOP]   Local PC URL:   http://localhost:8000/
   [MOBILE]    Phone / Wi-Fi:  http://192.168.X.X:8000/
   ```
3. Open `http://<YOUR_PC_IP>:8000/` in Safari or Chrome on your phone.
4. Tap **`Upload the pic`** to snap a photo with your phone's camera.
5. The PC will process the image, detect coins and colored bins, and display the results on your phone!

> [!TIP]
> If your phone cannot load the page, Windows Defender Firewall might be blocking incoming traffic. Open PowerShell as Administrator on your PC and run:
> ```powershell
> New-NetFirewallRule -DisplayName "Dum-E" -Direction Inbound -LocalPort 8000,8080 -Protocol TCP -Action Allow
> ```

---

## 🤖 ESP32 Hardware Wiring & Pinout

| Component | Pin (GPIO) | Signal Type | Description |
| :--- | :--- | :--- | :--- |
| **Base Servo (MG90S)** | **GPIO 13** | PWM (50 Hz) | Horizontal Yaw (0° to 180°) |
| **Shoulder Servo (MG90S)** | **GPIO 12** | PWM (50 Hz) | Vertical Pitch (0° to 180°) |
| **Elbow Servo (MG90S)** | **GPIO 14** | PWM (50 Hz) | Arm Flexion (0° to 180°) |
| **Wrist Servo (MG90S)** | **GPIO 27** | PWM (50 Hz) | Tilt Angle (0° to 180°) |
| **Electromagnet Relay** | **GPIO 26** | Digital Out | Active HIGH (Relay ON) |
| **Power Supply** | External 5V 3A+ | Power | **Connect Common Ground (GND) to ESP32!** |

> [!CAUTION]
> **Never power servos directly from the ESP32 3.3V or Vin pins!** Servos draw significant stall currents that will cause the ESP32 to brown out or damage its voltage regulator. Always use an external 5V 3A+ power supply with ground tied to ESP32 GND.

---

## 🛠️ ESP32 Firmware Flashing Guide (Arduino IDE)

1. Download and install [Arduino IDE 2.x](https://www.arduino.cc/en/software).
2. Go to **File > Preferences** and add the ESP32 board URL to **Additional boards manager URLs**:
   ```
   https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json
   ```
3. Open **Tools > Board > Boards Manager**, search for `esp32` by Espressif, and install it.
4. Go to **Sketch > Include Library > Manage Libraries**, search and install:
   - **ESP32Servo** (by Kevin Harrington / John K. Bennett)
   - **ArduinoJson** (v6 or v7 by Benoit Blanchon)
   - **ESPAsyncWebServer** and **AsyncTCP** (by me-no-dev / lacamera)
5. Open [`firmware/esp32_firmware.ino`](firmware/esp32_firmware.ino) in Arduino IDE.
6. Edit your Wi-Fi credentials on lines 23-24:
   ```cpp
   const char* ssid     = "Samung";
   const char* password = "12345678";
   ```
7. Select your board (e.g., **ESP32 Dev Module**) and USB COM port, then click **Upload**.
8. Open the **Serial Monitor** at **115200 baud** to see the assigned local IP address (e.g., `http://192.168.1.42`).
9. In the Command Center web dashboard, enter this IP into the **ESP32 IP** box.

---

## 🎯 How to Use the Command Center

```
+-------------------------------------------------------------+
|                    TOP: CENTERED IMAGE                      |
|           [Shows "NO IMAGE" until photo loaded]             |
|          * Tap anywhere to set Base Reference (0,0) *        |
+-------------------------------------------------------------+
| [Upload the pic] | [DETECT BINS] | [SET BASE] | [TO box 1]  |
| [Load Sample]    | (Refresh)     | (X, Y px)  | [to box 2]  |
|                  |               |            | [to box 3]  |
|                  |               |            | [Cancel]    |
|                  |               |            | [abort]     |
+-------------------------------------------------------------+
|                     all SLIDER CONTROLS                     |
|  Base: [---O---] 90°               Shoulder: [---O---] 90°  |
|  Elbow: [---O---] 90°              Wrist:    [---O---] 90°  |
|                                                             |
|  [ELECTROMAGNET: OFF]             [RESET NEUTRAL (90°)]     |
+-------------------------------------------------------------+
```

1. **Upload an Image:**
   - Click the centered **NO IMAGE** window or the **`Upload the pic`** button to choose a photo.
   - Or click **`Load Sample Workbench Pic`** for an instant demo with colored bins and coins.
2. **Set Robot Base Reference:**
   - **Tap anywhere on the image** where your robotic arm sits on the table.
   - A target marker labeled `BASE REF (0, 0)` will appear at that point.
   - All Inverse Kinematics calculations for every coin and bin will immediately recalculate relative to that point!
3. **Detect Colored Bins:**
   - Click **`🔍 DETECT BINS (REFRESH)`** to scan the image for colored sorting boxes (Green, Red, Black, Blue, Yellow).
   - Each detected bin will be highlighted with a solid colored border and ID (`Box 1`, `Box 2`, `Box 3`).
4. **Sort Objects:**
   - Click **`[TO box 1]`**, **`[to box2]`**, or **`[to box3]`** to move the robotic arm directly to the detected physical coordinates of that colored bin and release the coin.
5. **Emergency Abort:**
   - Click the red **`[abort]`** button at any time to instantly cut power to the electromagnet and center all servos to 90°.

---

## 🧪 Running Automated Tests

A comprehensive unit and integration test suite is included:

```bash
# Windows
.venv\Scripts\python -m unittest discover -s tests -p "test_*.py" -v

# Linux / macOS
python3 -m unittest discover -s tests -p "test_*.py" -v
```

This verifies:
- Planar and 3D Inverse Kinematics reachability and angle clamping.
- Relative kinematics calculation from tapped base coordinates.
- Hough Circles coin detection.
- Rectangular colored bin contour detection.
- ESP32 REST API endpoints (`/status`, `/joint`, `/grip`, `/mode`, `/set_base_tap`, `/detect_bins`, `/sort_to_box`, `/abort`).

---

## 📁 Repository Structure

```
├── firmware/
│   └── esp32_firmware.ino       # ESP32 Arduino sketch (servos + relay + AsyncWebServer)
├── src/
│   ├── inverse_kinematics.py    # 2D & 3D Inverse Kinematics engine + tap-to-set solver
│   ├── stream_server.py         # OpenCV vision pipeline, bin detection, and Flask API
│   └── esp32_mock.py            # Hardware-in-the-loop mock simulator
├── tests/
│   ├── test_ik.py               # Unit tests for kinematics solvers
│   ├── test_vision.py           # Unit tests for coin and bin detection
│   └── test_api.py              # Integration tests for REST endpoints
├── index.html                   # Dum-E responsive UI
├── run_server.py                # Unified launcher for Vision Server & Mock Hardware
├── requirements.txt             # Python dependencies
├── setup.bat                    # 1-Click setup installer for Windows
├── run.bat                      # 1-Click launcher for Windows
├── setup.sh                     # 1-Click setup script for Linux/macOS
├── run.sh                       # 1-Click launcher for Linux/macOS
└── README.md                    # Project documentation
```

---

## 📜 License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.
