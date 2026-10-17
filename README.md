# Anti-Gravity 4-DOF Robotic Arm & Vision Command Center

A complete end-to-end robotic arm control and computer vision system designed for 4-DOF servo arms (MG90S) with an electromagnet end-effector, ESP32 asynchronous HTTP control, OpenCV real-time coin detection (Hough Circles), planar & 3D Inverse Kinematics (IK), and a cyber-themed web command dashboard.

---

## Architecture Overview

```
                      +---------------------------------------+
                      |         Photo / Picture Upload        |
                      |    (JPG, PNG, WEBP via Drag & Drop)   |
                      +-------------------+-------------------+
                                          |
                                          v
+-----------------------+     +-----------+-----------+     +-----------------------+
|  Command Center UI    | <-> |  Python Vision Server | --> |   Inverse Kinematics  |
|     (index.html)      |     |   (stream_server.py)  |     |   (ArmKinematics 3D)  |
+-----------------------+     +-----------+-----------+     +-----------------------+
            |                             |
            | (Direct HTTP)               | (Autonomous Joint Target Dispatch)
            v                             v
+-----------------------------------------------------+
|           ESP32 Microcontroller (Wi-Fi)            |
|               (firmware/esp32_firmware.ino)         |
+-----------------------------------------------------+
  |          |               |             |          |
  v          v               v             v          v
Base      Shoulder         Elbow         Wrist   Electromagnet
(Pin 13)  (Pin 12)        (Pin 14)      (Pin 27) (Pin 26 Relay)
```

---

## Photo Upload & Detection Workflow

Instead of requiring an active RTSP video feed, you can take a photo of your workbench with your phone or camera and upload it directly:

1. **Upload Workbench Picture:** Drag and drop or browse to select your photo in the Command Center UI.
2. **OpenCV Hough Circles Processing:** The server immediately runs Gaussian/median filtering, detects circular coins, and draws cyan rings with center markers.
3. **Kinematics Calculation:** The 3D Inverse Kinematics engine calculates the real-world $(X, Y, Z)$ coordinates and determines servo angles (`Base`, `Shoulder`, `Elbow`, `Wrist`).
4. **Interactive Target Dispatch:** Click **MOVE ARM HERE** on any detected coin card to automatically command the robotic arm to reach that position.
5. **Instant Demo:** Click **✨ Load Sample Workbench Image** to test coin detection and kinematics with one click!

---

## Project Structure

```
├── firmware/
│   └── esp32_firmware.ino       # Complete ESP32 Arduino sketch (servos + relay + AsyncWebServer)
├── src/
│   ├── inverse_kinematics.py    # 2D & 3D Inverse Kinematics engine + pixel-to-world mapping
│   ├── stream_server.py         # OpenCV Hough Circles pipeline + MJPEG stream + Flask API
│   └── esp32_mock.py            # Hardware-in-the-loop mock server for offline testing
├── tests/
│   ├── test_ik.py               # Unit tests for kinematics reachability & angle mapping
│   ├── test_vision.py           # Unit tests for synthetic coin generation & Hough detection
│   └── test_api.py              # Integration tests for REST endpoints and auto-mode locks
├── index.html                   # Anti-Gravity Command Center modern HTML5 control dashboard
├── run_server.py                # Master runner (starts Vision Server & Mock Hardware)
└── requirements.txt             # Python dependencies
```

---

## Hardware Pinout (ESP32 DevKit v1)

| Component | Pin (GPIO) | Power / Signal | Notes |
| :--- | :--- | :--- | :--- |
| **Base Servo (MG90S)** | **GPIO 13** | PWM (50 Hz) | Horizontal Yaw (0° to 180°) |
| **Shoulder Servo (MG90S)** | **GPIO 12** | PWM (50 Hz) | Elevation Pitch (0° to 180°) |
| **Elbow Servo (MG90S)** | **GPIO 14** | PWM (50 Hz) | Forearm Flex (0° to 180°) |
| **Wrist Servo (MG90S)** | **GPIO 27** | PWM (50 Hz) | End-effector Tilt (0° to 180°) |
| **Electromagnet Relay** | **GPIO 26** | Digital Output | Active HIGH = Magnet ON |
| **Servo Power (External)** | External 5V 3A+ | VCC / GND | **Never power servos directly from ESP32 3.3V pin!** Common Ground with ESP32. |

---

## 1. ESP32 Flashing Instructions (Arduino IDE)

1. Open Arduino IDE.
2. In **Tools > Board > Boards Manager**, ensure `esp32` by Espressif Systems is installed.
3. In **Sketch > Include Library > Manage Libraries**, install:
   - **ESP32Servo** (by Kevin Harrington)
   - **ArduinoJson** (v6 or v7 by Benoit Blanchon)
   - **ESPAsyncWebServer** and **AsyncTCP** (by me-no-dev / lacamera)
4. Open [`firmware/esp32_firmware.ino`](file:///c:/Users/Milan%20Gaurav/OneDrive/Desktop/Arm/firmware/esp32_firmware.ino).
5. Update lines 23-24 with your Wi-Fi credentials:
   ```cpp
   const char* ssid     = "YOUR_WIFI_NAME";
   const char* password = "YOUR_WIFI_PASSWORD";
   ```
6. Select your ESP32 board and COM port, then click **Upload**.
7. Open **Serial Monitor** at **115200 baud** to see the assigned local IP address (e.g. `http://192.168.1.42`).

---

## 2. Running the Python Server & Test Environment

### Quick Start
To launch both the Vision Stream Server and the simulated ESP32 mock hardware:

```bash
.\.venv\Scripts\python.exe run_server.py --synthetic
```

Then open your browser to:
- **Command Dashboard:** [`http://localhost:8000/`](http://localhost:8000/)
- **Live Video Stream:** [`http://localhost:8000/video_feed`](http://localhost:8000/video_feed)
- **Simulated Hardware API:** [`http://localhost:8080/status`](http://localhost:8080/status)

### Connecting to Real Hardware
Once your ESP32 is powered and on your Wi-Fi, run:

```bash
.\.venv\Scripts\python.exe run_server.py --no-mock --esp-ip "http://192.168.1.42"
```
Or simply type the IP directly into the **ESP32 IP** box in the web dashboard header!

### Connecting an RTSP Phone Camera Feed
1. Install an RTSP app on your smartphone (e.g., **IP Webcam** on Android or **RTSP Camera** on iOS).
2. Start the camera server on your phone and note the URL (e.g. `rtsp://192.168.1.105:8554/`).
3. Set the environment variable and run:
   ```bash
   $env:RTSP_URL = "rtsp://192.168.1.105:8554/"
   .\.venv\Scripts\python.exe run_server.py --no-mock
   ```

---

## 3. Running Automated Tests

All units (Inverse Kinematics, OpenCV circle detection, and REST APIs) are fully covered:

```bash
.\.venv\Scripts\python.exe -m unittest discover -s tests -p "test_*.py" -v
```

---

## 4. Tuning Inverse Kinematics & Vision

- **Arm Link Lengths:** Measure your 3D-printed links from pivot-to-pivot and configure in `src/inverse_kinematics.py`:
  - `l1`: Distance from shoulder pivot to elbow pivot in mm (default: `120.0`).
  - `l2`: Distance from elbow pivot to wrist/magnet pivot in mm (default: `120.0`).
- **Hough Circle Tuning:**
  - `param1`: Sensitivity for Canny edge detector (decrease if edges are missed, increase to remove noise).
  - `param2`: Accumulator threshold for circle centers (lower finds more circles, higher requires stricter circularity).
