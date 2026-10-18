# PROJECT HANDOFF: DUM-E 3-DOF ROBOTIC ARM & VISION COMMAND DECK

---

## 1. Overview
* **Goal**: Autonomous and manual 3-DOF robotic arm pick-and-place system that visually detects circular metallic coins and colored sorting bins from workbench photos, computes 3D Inverse Kinematics relative to a user-calibrated base origin, and actuates MG90S servos and an electromagnet relay over Wi-Fi.
* **Current Status**: **WORKING**. 15/15 automated unit and integration tests passing (`tests/`). Computer vision, IK solver, ESP32 REST server, hardware-in-the-loop mock, mobile web dashboard, cross-device auto-sync, and physical servo mapping are fully functional.

---

## 2. Architecture

### System Topology
* **ESP32 Firmware**: Standalone HTTP REST server on port 80 controlling PWM hardware timers and relay GPIO.
* **Interface / Backend**: Python 3.9–3.11 Flask server on port 8000 handling OpenCV vision, 3D analytical IK, motion sequencing state machines, and ESP32 REST proxying.
* **Client**: Responsive single-page web application (`web/index.html`) running on desktop or mobile browser.
* **Mock Simulator**: In-process Flask mock (`src/esp32_mock.py`) on port 8080 simulating hardware registers for full offline testing.

### Link Specifications
* **Transport**: HTTP/1.1 over Wi-Fi (802.11 b/g/n, 2.4 GHz).
* **Payload Format**: JSON (`application/json`) for control and telemetry; `multipart/form-data` for image upload.
* **Telemetry Polling Rate**: 
  * Heartbeat / status: 3000 ms (`checkConnection()`).
  * Image synchronization: 2000 ms (`syncImageFromServer()`).
  * Trajectory progress during active execution: 200 ms (`motionPollInterval`).
* **Manual Slider Throttle**: 50 ms client-side rate limit (`sliderThrottle`).

### ASCII Data-Flow Diagram
```
  +-------------------------------------------------------------------------+
  |                             MOBILE / DESKTOP                            |
  |                           Browser (web/index.html)                      |
  +-------------------------------------------------------------------------+
       | Camera Photo (multipart/form)        ^ Telemetry / JSON (HTTP)
       v                                      |
  +-------------------------------------------------------------------------+
  |                           FLASK SERVER (Port 8000)                      |
  |                            src/stream_server.py                         |
  |  - OpenCV Vision Pipeline (Hough Circles + HSV Bin Contours)            |
  |  - Analytical 3D IK Engine (src/inverse_kinematics.py)                  |
  |  - 6-Stage Trajectory Sequencer (Hover -> Pick -> Lift -> Transit ...)  |
  |  - Physical Calibration Inversion Layer (map_to_physical_joints)       |
  |  - Proxy Layer (bypasses browser CORS & Private Network Access blocks)  |
  +-------------------------------------------------------------------------+
       |                                      ^
       | HTTP POST /move, /joint, /grip       | HTTP GET /status (JSON)
       v                                      |
  +-------------------------------------------------------------------------+
  |                      ESP32 CONTROLLER (Port 80)                         |
  |                 firmware/esp32_firmware/esp32_firmware.ino              |
  |  - AsyncWebServer on port 80                                            |
  |  - ESP32Servo PWM hardware timers (50 Hz, 500-2400 us)                  |
  |  - Independent per-joint rate limiter & slow descent profile            |
  |  - Relay driver (GPIO 26)                                               |
  +-------------------------------------------------------------------------+
       | PWM (D13)         | PWM (D14)         | PWM (D27)       | Digital (D26)
       v                   v                   v                 v
  [Base MG90S]      [Shoulder MG90S]     [Elbow MG90S]     [Electromagnet]
```

---

## 3. Hardware

### Components
* 1x ESP32 DevKit v1 (30-pin, ESP-WROOM-32).
* 3x TowerPro MG90S Micro Servos (metal gear, 180° rotation, 4.8V–6.0V).
* 1x 5V Electromagnet Relay Module (active HIGH trigger).
* 1x 5V DC Electromagnet (suction latch for metallic coins).
* 1x External DC Power Supply (5V, minimum 3.0A recommended).
* 1x Common Ground bus tying ESP32 GND to External 5V PSU GND.

### Pin Map
| Pin (GPIO) | Board Label | Device Connected | Function | Safe Range / Mode |
| :--- | :--- | :--- | :--- | :--- |
| **GPIO 13** | D13 | Base Servo (MG90S) | Horizontal Yaw rotation | 0° to 180° (90° = Forward) |
| **GPIO 14** | D14 | Shoulder Servo (MG90S) | Arm Elevation / Pitch | 0° to 180° (90° = Upright) |
| **GPIO 27** | D27 | Elbow Servo (MG90S) | Arm Flexion / Forearm Reach | 0° to 180° (90° = Neutral) |
| **GPIO 26** | D26 | Electromagnet Relay | Power switching for magnet | Digital Output (HIGH=ON, LOW=OFF) |
| **GPIO 12** | D12 | **DO NOT CONNECT** | MTDI Strapping Pin | Must float at boot (avoids flash brownout) |
| **GND** | GND | Common Power Bus | Logic & Power Reference | Tied directly to external 5V PSU ground |

### Power & Wiring Notes
* **External 5V Isolation**: Servos and electromagnet must **never** draw power from the ESP32 3.3V or Vin rail. Stalling an MG90S draws up to 800 mA; three servos plus relay exceed 2.5A peak, inducing instantaneous ESP32 brownout resets.
* **Common Ground**: External 5V PSU ground and ESP32 GND must be physically connected, otherwise PWM reference floats and servos exhibit erratic jitter.
* **GPIO 12 Warning**: GPIO 12 (MTDI) controls internal flash voltage during boot. Pulling it HIGH during startup causes flash voltage to latch to 1.8V instead of 3.3V, bricking boot. Left floating/unconnected.

### Safe Servo Limits
* Physical rotation limit: 0° to 180° hard stop.
* Clamping enforced in both firmware (`constrain(angle, 0, 180)`) and IK solver (`max(0, min(180, ...))`).

---

## 4. Firmware

* **Source File**: [`firmware/esp32_firmware/esp32_firmware.ino`](file:///c:/Users/Milan%20Gaurav/OneDrive/Desktop/Arm/firmware/esp32_firmware/esp32_firmware.ino)
* **Libraries Required**:
  * `ESPAsyncWebServer` (by me-no-dev / lacamera)
  * `AsyncTCP` (by me-no-dev)
  * `ArduinoJson` (v6 or v7 by Benoit Blanchon)
  * `ESP32Servo` (by Kevin Harrington / John K. Bennett)
* **Serial Baud Rate**: `115200` baud.
* **PWM Allocation**: 3 timers allocated via `ESP32PWM::allocateTimer(0..2)` at 50 Hz, 500 µs to 2400 µs pulse widths.
* **Startup Initialization**:
  * Staggered attach: Base attached -> write(90) -> delay(150ms) -> Shoulder attached -> write(90) -> delay(150ms) -> Elbow attached -> write(90) -> delay(150ms).
  * Magnet initialized to `LOW` (OFF).
* **Communication Handling**:
  * `POST /joint`: Fast single-joint update. Payload: `{"joint": 0..2, "angle": 0..180}`. Bypasses smooth stepping for instant manual slider tracking.
  * `POST /move`: Coordinated 3-joint target update. Payload: `{"base": 0..180, "shoulder": 0..180, "elbow": 0..180, "elbowDelay": ms, "transitDelay": ms}`.
  * `POST /grip`: Controls electromagnet. Payload: `{"state": "on"|"close"|"off"|"open"}`.
  * `POST /mode`: Toggles `{"mode": "auto"|"manual"}`.
  * `GET /status`: Returns JSON snapshot of current angles, target angles, motion state, and magnet status.
  * `OPTIONS /*`: Injects CORS headers (`Access-Control-Allow-Origin: *`).
* **Main Loop (`loop()`)**:
  * Non-blocking millis() stepped motion engine for coordinated targets.
  * Base & Shoulder default rate: 20 ms per 1° increment.
  * Elbow asymmetric profile: Slow descent at 45 ms per 1° increment (`targetElbow > currentElbow`) for gentle table/coin contact; fast ascent at 25 ms per 1° increment.

---

## 5. Interface & Control Software

* **Backend Stack**: Python 3, Flask, OpenCV (`cv2`), NumPy, Requests.
* **Frontend Stack**: Single-page vanilla HTML5 / CSS3 / ES6+ JavaScript (`web/index.html`). No build tools or Node.js runtime required at runtime.
* **Kinematics & Coordinate System** ([`src/inverse_kinematics.py`](file:///c:/Users/Milan%20Gaurav/OneDrive/Desktop/Arm/src/inverse_kinematics.py)):
  * **Link Lengths**: $L_1$ (Shoulder-to-Elbow) = 120.0 mm, $L_2$ (Elbow-to-Tip) = 120.0 mm.
  * **Base Pivot Height**: $H_{\text{base}} = 50.0$ mm (configured in solver).
  * **Ground Scale**: Default 0.5 mm/pixel.
  * **Planar Reach**: $r = \sqrt{x^2 + y^2}$. Reachable condition: $|L_1 - L_2| \le r \le (L_1 + L_2)$ (0 to 240 mm).
  * **Formulas**:
    $$\theta_2 = \arccos\left(\frac{r^2 + z_{\text{eff}}^2 - L_1^2 - L_2^2}{2 L_1 L_2}\right)$$
    $$\theta_1 = \text{atan2}(z_{\text{eff}}, r) - \text{atan2}(L_2 \sin\theta_2, L_1 + L_2 \cos\theta_2)$$
    $$\theta_{\text{base}} = 90^\circ + \text{atan2}(\Delta x, \Delta y)$$
* **Physical Mapping Inversion Layer** ([`src/stream_server.py`](file:///c:/Users/Milan%20Gaurav/OneDrive/Desktop/Arm/src/stream_server.py)):
  * `base` = direct `[0, 180]` (90° = centered forward).
  * `shoulder` = inverted: `180 - shoulder_ik` (to accommodate reversed mechanical horn orientation).
  * `elbow` = direct `[0, 180]` (0° = back, 90° = upright, 180° = forward reach).
* **Control Modes**:
  1. **Manual Real-Time Telemetry**: Real-time slider bars with 50 ms throttling commanding `/api/joint`.
  2. **Tap-to-Set Base Reference**: User taps physical arm base location on camera frame; commits with "Set Base". All subsequent coin/bin reaches are computed relative to $(\Delta x, \Delta y)$ from that tapped origin.
  3. **Automated 6-Step Subway Progression**:
     * Step 1: `POST /api/step/hover_coin` (Hover 65° safe elevation over coin).
     * Step 2: `POST /api/step/pick_coin` (Slow descent, latch magnet).
     * Step 3: `POST /api/step/lift_coin` (Ascend to transit altitude).
     * Step 4: `POST /api/step/transit_box` (Steady transit to selected Box 1, 2, or 3).
     * Step 5: `POST /api/step/drop_coin` (Slow descent, open relay, release coin, ascend).
     * Step 6: `POST /api/step/home` (Return to 90° neutral).
  4. **Emergency Abort**: `POST /api/abort` drops relay immediately and forces Base, Shoulder, Elbow to 90°.

---

## 6. End-to-End Execution Pipeline

1. **Input**: User clicks photo using mobile phone camera or desktop webcam (`/api/upload`).
2. **Vision Processing**:
   * Preprocessing: Gaussian Blur (`ksize=7`), HSV conversion.
   * Coin detection: `cv2.HoughCircles` (`dp=1.2`, `minDist=55`, `param1=50`, `param2=30`, `minRadius=15`, `maxRadius=50`).
   * Bin detection: HSV thresholding for GREEN, RED (dual band), BLUE, BLACK, YELLOW. Area filter $\ge 8000\text{ px}^2$, aspect-ratio gate $\ge 1.25$ or bbox fill $\ge 0.65$ to reject round coins.
3. **Kinematics Calculation**:
   * Bins and coins mapped from pixel space to real-world mm relative to base pixel.
   * Analytical 3D IK computes `(base, shoulder, elbow)` angles.
4. **Command Generation**:
   * Angles passed through `map_to_physical_joints()` to invert shoulder horn.
5. **Actuation**:
   * Flask backend issues HTTP POST requests to ESP32 IP (`http://10.34.233.78/move` or `/joint`).
   * ESP32 PWM channels drive physical servos with smooth step delays.

### Build, Flash & Run Commands

```cmd
:: 1. Flash ESP32 Firmware
:: Open firmware/esp32_firmware/esp32_firmware.ino in Arduino IDE 2.x
:: Select Board: "ESP32 Dev Module", correct COM port
:: Click Upload. Check Serial Monitor at 115200 baud for IP address.

:: 2. Automated 1-Click Install on Host PC (Windows)
scripts\setup.bat

:: 3. Run Server & Control Deck
scripts\run.bat

:: Linux / macOS equivalent:
chmod +x scripts/setup.sh scripts/run.sh
./scripts/setup.sh
./scripts/run.sh

:: 4. Run Automated Test Suite
.\.venv\Scripts\python.exe -m unittest discover tests -v
```

---

## 7. Feature Status Matrix

| Feature | Primary File | Status | Notes |
| :--- | :--- | :--- | :--- |
| 3-DOF Servo PWM Control | `firmware/esp32_firmware/esp32_firmware.ino` | **WORKING** | MG90S Base (D13), Shoulder (D14), Elbow (D27). |
| Electromagnet Relay Driver | `firmware/esp32_firmware/esp32_firmware.ino` | **WORKING** | GPIO 26, active HIGH. |
| ESP32 REST API Server | `firmware/esp32_firmware/esp32_firmware.ino` | **WORKING** | Endpoints `/status`, `/joint`, `/move`, `/grip`, `/mode`. |
| 3D Analytical Inverse Kinematics | `src/inverse_kinematics.py` | **WORKING** | Closed-form trigonometry with reach bounds. |
| Tap-to-Set Base Reference | `src/inverse_kinematics.py`, `web/index.html` | **WORKING** | Decoupled staging + commit without servo movement. |
| OpenCV Hough Circles Coin Detection | `src/stream_server.py` | **WORKING** | Sub-pixel circle detection with parameter tuning. |
| Rectangular Colored Bin Detection | `src/stream_server.py` | **WORKING** | Green, Red, Blue, Black, Yellow with coin rejection. |
| Hardware Simulator (Mock ESP32) | `src/esp32_mock.py` | **WORKING** | Port 8080 simulation for offline testing without hardware. |
| REST API Proxy Layer | `src/stream_server.py` | **WORKING** | Bypasses mobile browser PNA / CORS security blocks. |
| 6-Stage Subway Step Sequencer | `src/stream_server.py`, `web/index.html` | **WORKING** | Hover -> Pick -> Lift -> Transit -> Drop -> Home. |
| Cross-Device Auto Sync | `src/stream_server.py`, `web/index.html` | **WORKING** | Phone upload automatically updates PC screen. |
| Dynamic Working Link Resolver | `src/stream_server.py`, `web/index.html` | **WORKING** | Auto-detects local Wi-Fi IP and active port for phone link. |
| Emergency Abort Interlock | `src/stream_server.py`, `web/index.html` | **WORKING** | Immediate relay drop and 90° neutral reset. |

---

## 8. Bugs and Fixes (Chronological)

| # | Symptom | Root Cause | Fix | File(s) Affected |
| :---: | :--- | :--- | :--- | :--- |
| **1** | ESP32 failed to boot when third servo was connected to GPIO 12. | GPIO 12 is the MTDI strapping pin; pulling it HIGH during boot sets internal SPI flash voltage to 1.8V instead of 3.3V, crashing ROM bootloader. | Shifted third servo to GPIO 27. Left GPIO 12 completely unconnected. | `firmware/esp32_firmware/esp32_firmware.ino` |
| **2** | Servos jittered violently and brownout-reset the ESP32 when commanded simultaneously. | Inrush current from 3 stalling MG90S servos exceeded USB 500mA limit; floating reference without common ground. | Mandated external 5V 3A+ PSU with common ground bus tied to ESP32 GND; added staggered startup attach in firmware with 150ms delays. | `firmware/esp32_firmware/esp32_firmware.ino` |
| **3** | Wi-Fi connection timed out on boot. | SSID typo in configuration (`"Samung"` instead of `"Samsung"`). | Corrected SSID string constant to `"Samsung"`. | `firmware/esp32_firmware/esp32_firmware.ino` |
| **4** | Sliders suffered 1–2 second lag and stutter during manual drag. | Manual slider calls routed through `/move` which applied multi-second smooth stepping delays. | Re-routed `/joint` in firmware to execute direct instant servo write; added 50 ms client-side throttling in JavaScript. | `firmware/esp32_firmware/esp32_firmware.ino`, `web/index.html` |
| **5** | Elbow moved in reverse (reaching backward when commanded forward). | Mechanical horn on elbow servo was physically mounted in direct [0, 180] orientation, but UI had inverted subtraction. | Removed UI inversion; unified Elbow as direct [0, 180] (0° = Back, 180° = Forward Reach). | `web/index.html`, `src/stream_server.py` |
| **6** | Shoulder moved backward during automated pick-and-place routines. | Shoulder servo horn is mechanically mirrored relative to the Cartesian IK coordinate frame. | Added `map_to_physical_joints()` in server: `shoulder_cmd = 180 - shoulder_ik`. Base and Elbow left direct. | `src/stream_server.py` |
| **7** | Mobile phone browser could not control arm or query status (`Failed to fetch`). | Modern mobile browsers enforce Private Network Access (PNA) and CORS, blocking public/private origin requests to raw ESP32 IP (`http://10.34.233.78`). | Implemented backend Flask proxies (`/api/joint`, `/api/status`, `/api/grip`, `/api/step/*`) on PC port 8000. Phone talks only to PC server. | `src/stream_server.py`, `web/index.html` |
| **8** | Dark centers of circular metallic coins falsely detected as "Box (BLACK)" bins. | Shiny coin core shadow produced low value ($V < 55$) matching loose black HSV range, and area gate was too low ($2000\text{ px}^2$). | Raised area threshold to $\ge 8000\text{ px}^2$; added aspect-ratio circularity filter ($\text{aspect} < 1.25$ and fill $< 0.65$ rejected); tightened black $V \le 45, S \le 60$. | `src/stream_server.py` |
| **9** | Tapping image to calibrate base reference caused unexpected arm motion. | `handleImageTap()` and `renderResults()` triggered `updateSlidersDisplay()`, which had active event listeners firing `/api/joint`. | Decoupled staging from execution: tapping only stages $(X, Y)$ into text field; operator must click "Set Base" to commit calibration; stripped actuation hooks from display updater. | `web/index.html` |
| **10** | Photos taken on mobile phone did not update on the desktop monitor. | PC browser had no push/poll notification for uploads initiated from external clients. | Added global `image_version` counter in Flask bumped on `/api/upload`; added 2-second client sync polling (`syncImageFromServer()`) in frontend. | `src/stream_server.py`, `web/index.html` |
| **11** | Double-clicking batch scripts inside `scripts/` failed to find `.venv` or server files. | Scripts executed with working directory set to `scripts/` rather than repository root. | Added `cd /d "%~dp0.."` to `.bat` files and `cd "$(dirname "$0")/.."` to `.sh` files to guarantee root execution. | `scripts/run.bat`, `scripts/setup.bat`, `scripts/push.bat`, `scripts/run.sh`, `scripts/setup.sh` |
| **12** | Mobile users unable to connect when local Wi-Fi router reassigned PC's IP via DHCP. | Users had to run `ipconfig` manually to find PC LAN IP; hostname was hardcoded to `localhost`. | Created `/api/network_info` using UDP socket probe to detect active Wi-Fi IP and port; rendered `WEBSITE WORKING LINK` in header with 1-click copy button. | `src/stream_server.py`, `web/index.html`, `tests/test_api.py` |
| **13** | Setting base reference reverted photo to synthetic sample image, and cross-device sync failed. | `/api/detect` implicitly called `get_sample()` when uninitialized, wiping `current_raw_image` with fake workbench graphic. Neither `/api/set_base_tap` nor `/api/detect_bins` incremented `image_version`. Raw mobile camera taps suffered letterbox offset under CSS `object-fit: contain`. | Removed implicit sample fallback from `/api/detect` (returns `no_image`); added `image_version += 1` to `/api/set_base_tap` and `/api/detect_bins`; implemented letterbox/pillarbox coordinate offset compensation in `handleImageTap()`; added `isSyncing` re-entrancy lock in client. | `src/stream_server.py`, `web/index.html`, `tests/test_api.py` |

---

## 9. Current Calibration & Tuning Values

### Geometry & Kinematics
* Shoulder-to-Elbow Length ($L_1$): `120.0 mm`
* Elbow-to-Tip Length ($L_2$): `120.0 mm`
* Base-to-Shoulder Height ($H_{\text{base}}$): `50.0 mm`
* Default Ground Resolution: `0.5 mm/pixel`
* Camera Origin Offset: `(0.0, 150.0) mm`
* Coin Elevation ($Z_{\text{coin}}$): `10.0 mm`
* Bin Elevation ($Z_{\text{bin}}$): `30.0 mm`
* Safe Transit Elevation (Elbow): `65°`

### Physical Joint Calibration Mappings
* Base (Joint 0): Direct $[0, 180]^\circ$ (90° = Neutral / Straight Ahead).
* Shoulder (Joint 1): Inverted: $\theta_{\text{phys}} = 180 - \theta_{\text{ik}}$ (90° = Vertical Upright, 0° = Retracted Back, 180° = Full Forward Reach).
* Elbow (Joint 2): Direct $[0, 180]^\circ$ (90° = Right Angle, 0° = Back, 180° = Full Forward Reach).

### Motion Delays
* Base Smooth Step Delay: `20 ms/deg`
* Shoulder Smooth Step Delay: `20 ms/deg`
* Elbow Descent Delay (Downwards): `45 ms/deg`
* Elbow Ascent Delay (Upwards): `25 ms/deg`
* Latch Settle Delay: `400 ms`
* Release Settle Delay: `400 ms`

### Vision Detection Parameters
* **Hough Circles (Coins)**: `dp=1.2`, `minDist=55`, `param1=50`, `param2=30`, `minRadius=15`, `maxRadius=50`, `blurKernel=7`.
* **Colored Bins (HSV Ranges)**:
  * GREEN: `[35, 100, 80]` to `[88, 255, 255]`
  * RED 1: `[0, 100, 80]` to `[10, 255, 255]`
  * RED 2: `[170, 100, 80]` to `[180, 255, 255]`
  * BLUE: `[90, 70, 50]` to `[135, 255, 255]`
  * BLACK: `[0, 0, 0]` to `[180, 60, 45]`
  * YELLOW: `[15, 70, 70]` to `[35, 255, 255]`
  * Minimum Bin Area: `8000 px²`
  * Aspect Ratio / Fill Gate: reject if $\text{aspect} < 1.25$ AND $\text{bbox fill} < 0.65$.

### Fallback Bin Joint Coordinates (when unassisted by vision)
* Box 1: `{"base": 35, "shoulder": 40, "elbow": 90}`
* Box 2: `{"base": 90, "shoulder": 35, "elbow": 90}`
* Box 3: `{"base": 145, "shoulder": 40, "elbow": 90}`

---

## 10. Known Issues and Next Steps (Prioritized)

1. **Priority 1: Lighting Sensitivity in Vision Thresholds**
   * *Issue*: Under extreme yellow incandescent or very dim workbench lighting, yellow and green bins can cross-talk or metallic coin reflections can miss Hough circle accumulator thresholds.
   * *Next Step*: Implement automatic white balancing (Gray World algorithm) and adaptive thresholding prior to HSV masking.
2. **Priority 2: Z-Height Variation Across Workbench**
   * *Issue*: Target Z height is currently constant (`10.0 mm` for coins, `30.0 mm` for bins). If the table surface is tilted or camera is angled, the magnet may stop 2–3 mm above the coin.
   * *Next Step*: Add interactive Z-trim offset slider on the UI to adjust end-effector touch-down depth in real-time.
3. **Priority 3: Static IP or mDNS on ESP32**
   * *Issue*: ESP32 IP is assigned dynamically by DHCP (`10.34.233.78`). If the router reassigns the ESP32 IP, the operator must re-enter it in the header.
   * *Next Step*: Enable `ESPmDNS` in firmware so the arm is accessible at `http://dume.local/` without tracking IP changes.
4. **Priority 4: Trajectory Interruption on Abort**
   * *Issue*: If the multi-stage pick-and-place thread is active, calling `/api/abort` sends neutral commands but the background thread may still issue its next scheduled step before terminating.
   * *Next Step*: Add an explicit `threading.Event()` cancel token checked before every stage in `run_smooth_pick_and_drop_sequence()`.

---

## 11. Quick Start from Scratch (Minimal Steps)

```cmd
:: Step 1: Clone repo
git clone https://github.com/shashankbm666/Dum-E.git
cd Dum-E

:: Step 2: Run automated installer
scripts\setup.bat

:: Step 3: Flash ESP32 (if hardware present)
:: Open firmware/esp32_firmware/esp32_firmware.ino in Arduino IDE, edit Wi-Fi, upload.

:: Step 4: Launch Command Deck (starts Vision Server on :8000 + Mock ESP32 on :8080)
scripts\run.bat

:: Step 5: Open Browser
:: Desktop: http://localhost:8000/
:: Phone: Open the link shown in the "WEBSITE WORKING LINK" header pill.
```

---

## Read This First

1. **Power Supply**: Always power the servos from an external 5V 3A+ power supply with ground tied to ESP32 GND; never power servos from the ESP32 board directly.
2. **Strapping Pin Safety**: Never connect any servo or wire to **GPIO 12 (D12)** — it will cause the ESP32 flash voltage to latch to 1.8V and brick boot.
3. **Physical Calibration**: The shoulder servo horn is inverted ($\theta_{\text{phys}} = 180 - \theta_{\text{ik}}$); Base and Elbow are direct ($0^\circ$ back, $90^\circ$ upright, $180^\circ$ forward reach).
4. **Mobile Browser Access**: Access the UI on your phone using the exact link displayed in the **`WEBSITE WORKING LINK`** header pill; do not call the ESP32 directly from your phone (browser CORS/PNA will block it).
5. **Base Calibration**: Tapping the photo stages the base origin; you must press **"Set Base"** to commit the reference point, which recalibrates kinematics without moving the servos.
