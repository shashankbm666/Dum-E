/*
 * ESP32 Robotic Arm & Vision Command Center Firmware
 * ----------------------------------------------------
 * Controls 4x MG90S Servos (Base, Shoulder, Elbow, Wrist) and an Electromagnet Relay
 * Exposes a RESTful HTTP API over local Wi-Fi.
 *
 * Required Arduino Libraries:
 *  - ESPAsyncWebServer (by me-no-dev / lacamera)
 *  - AsyncTCP (by me-no-dev)
 *  - ArduinoJson (v6 or v7 by Benoit Blanchon)
 *  - ESP32Servo (by Kevin Harrington / John K. Bennett)
 */

#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <ArduinoJson.h>
#include <ESP32Servo.h>

// ==========================================
// Network Configuration
// REPLACE WITH YOUR WIFI CREDENTIALS
// ==========================================
const char* ssid     = "Samung";
const char* password = "12345678";

// ==========================================
// Hardware Pin Mapping (ESP32 DevKit v1)
// ==========================================
const int BASE_PIN     = 13;  // Base Rotation Servo
const int SHOULDER_PIN = 12;  // Shoulder Pitch Servo
const int ELBOW_PIN    = 14;  // Elbow Flex Servo
const int WRIST_PIN    = 27;  // Wrist Tilt Servo
const int MAGNET_PIN   = 26;  // Electromagnet Relay Module

// Servo Objects
Servo baseServo;
Servo shoulderServo;
Servo elbowServo;
Servo wristServo;

// Web Server on port 80
AsyncWebServer server(80);

// Hardware Position State (Current Physical Angles)
int currentBase     = 90;
int currentShoulder = 90;
int currentElbow    = 90;
int currentWrist    = 90;

// Target Angles (Commanded by user or automated routines)
int targetBase     = 90;
int targetShoulder = 90;
int targetElbow    = 90;
int targetWrist    = 90;

// Aliases for compatibility
int baseAngle     = 90;
int shoulderAngle = 90;
int elbowAngle    = 90;
int wristAngle    = 90;

bool magnetOn     = false;
bool autoMode     = false;

// Smooth Stepping Delays (ms per 1-degree step)
// Base & Shoulder: 20ms per degree (smooth and steady across bins)
int baseStepDelay     = 20;
int shoulderStepDelay = 20;

// Elbow servo: Extra-slow descent to pick or drop coin gently (45ms per deg), steady ascent (25ms per deg)
int elbowDownDelay    = 45;
int elbowUpDelay      = 25;

int wristStepDelay    = 20;

// Timers for rate limiting
unsigned long lastBaseTick     = 0;
unsigned long lastShoulderTick = 0;
unsigned long lastElbowTick    = 0;
unsigned long lastWristTick    = 0;

// Helper to inject CORS headers for browser frontends
void addCORS(AsyncWebServerResponse* res) {
    res->addHeader("Access-Control-Allow-Origin", "*");
    res->addHeader("Access-Control-Allow-Methods", "POST, GET, OPTIONS");
    res->addHeader("Access-Control-Allow-Headers", "Content-Type");
}

void setup() {
    Serial.begin(115200);
    delay(500);
    Serial.println("\n[SYSTEM] Booting ESP32 Robotic Arm Controller...");

    // Configure Relay Pin
    pinMode(MAGNET_PIN, OUTPUT);
    digitalWrite(MAGNET_PIN, LOW); // Electromagnet starts OFF

    // Allocate PWM timers for ESP32Servo
    ESP32PWM::allocateTimer(0);
    ESP32PWM::allocateTimer(1);
    ESP32PWM::allocateTimer(2);
    ESP32PWM::allocateTimer(3);

    // Initialize Servos with standard 50Hz and 500-2400us pulse widths for MG90S
    baseServo.setPeriodHertz(50);
    baseServo.attach(BASE_PIN, 500, 2400);
    baseServo.write(baseAngle);

    shoulderServo.setPeriodHertz(50);
    shoulderServo.attach(SHOULDER_PIN, 500, 2400);
    shoulderServo.write(shoulderAngle);

    elbowServo.setPeriodHertz(50);
    elbowServo.attach(ELBOW_PIN, 500, 2400);
    elbowServo.write(elbowAngle);

    wristServo.setPeriodHertz(50);
    wristServo.attach(WRIST_PIN, 500, 2400);
    wristServo.write(wristAngle);

    Serial.println("[SYSTEM] Servos centered at 90 degrees.");

    // Connect to Wi-Fi
    Serial.printf("[WIFI] Connecting to %s ", ssid);
    WiFi.mode(WIFI_STA);
    WiFi.begin(ssid, password);
    
    int attempts = 0;
    while (WiFi.status() != WL_CONNECTED && attempts < 40) {
        delay(500);
        Serial.print(".");
        attempts++;
    }

    if (WiFi.status() == WL_CONNECTED) {
        Serial.println("\n[WIFI] Connected successfully!");
        Serial.print("[WIFI] IP Address: ");
        Serial.println(WiFi.localIP().toString());
    } else {
        Serial.println("\n[WIFI] Failed to connect to Wi-Fi. Check credentials or AP.");
    }

    // ----------------------------------------------------
    // HTTP API Endpoints
    // ----------------------------------------------------

    // Handle CORS preflight options
    server.on("/*", HTTP_OPTIONS, [](AsyncWebServerRequest* req) {
        AsyncWebServerResponse* res = req->beginResponse(204);
        addCORS(res);
        req->send(res);
    });

    // Root status page
    server.on("/", HTTP_GET, [](AsyncWebServerRequest* req) {
        String msg = "{\"device\":\"ESP32-Robotic-Arm\",\"status\":\"online\",\"ip\":\"" + WiFi.localIP().toString() + "\"}";
        AsyncWebServerResponse* res = req->beginResponse(200, "application/json", msg);
        addCORS(res);
        req->send(res);
    });

    // Query Arm State
    server.on("/status", HTTP_GET, [](AsyncWebServerRequest* req) {
        #if ARDUINOJSON_VERSION_MAJOR >= 7
            JsonDocument doc;
        #else
            StaticJsonDocument<384> doc;
        #endif
        bool isMoving = (currentBase != targetBase || currentShoulder != targetShoulder ||
                         currentElbow != targetElbow || currentWrist != targetWrist);

        doc["status"] = "ok";
        doc["base"] = currentBase;
        doc["shoulder"] = currentShoulder;
        doc["elbow"] = currentElbow;
        doc["wrist"] = currentWrist;
        doc["targetBase"] = targetBase;
        doc["targetShoulder"] = targetShoulder;
        doc["targetElbow"] = targetElbow;
        doc["targetWrist"] = targetWrist;
        doc["isMoving"] = isMoving;
        doc["magnet"] = magnetOn;
        doc["autoMode"] = autoMode;

        String response;
        serializeJson(doc, response);
        AsyncWebServerResponse* res = req->beginResponse(200, "application/json", response);
        addCORS(res);
        req->send(res);
    });

    // Set Joint Angle with Smooth Stepping (0=Base, 1=Shoulder, 2=Elbow, 3=Wrist)
    server.on("/joint", HTTP_POST, [](AsyncWebServerRequest* req){}, NULL,
        [](AsyncWebServerRequest* req, uint8_t* data, size_t len, size_t index, size_t total) {
            if (autoMode) {
                AsyncWebServerResponse* res = req->beginResponse(403, "application/json", "{\"error\":\"Auto mode active\"}");
                addCORS(res);
                req->send(res);
                return;
            }

            #if ARDUINOJSON_VERSION_MAJOR >= 7
                JsonDocument doc;
            #else
                StaticJsonDocument<256> doc;
            #endif
            DeserializationError err = deserializeJson(doc, data, len);
            if (err) {
                AsyncWebServerResponse* res = req->beginResponse(400, "application/json", "{\"error\":\"Invalid JSON\"}");
                addCORS(res);
                req->send(res);
                return;
            }

            int joint = doc["joint"];
            int angle = constrain((int)doc["angle"], 0, 180);

            // Set target - the loop() smoothly glides to this angle without jerky snaps
            switch (joint) {
                case 0: targetBase = angle; break;
                case 1: targetShoulder = angle; break;
                case 2: targetElbow = angle; break;
                case 3: targetWrist = angle; break;
                default:
                    AsyncWebServerResponse* res = req->beginResponse(400, "application/json", "{\"error\":\"Unknown joint index\"}");
                    addCORS(res);
                    req->send(res);
                    return;
            }

            Serial.printf("[SMOOTH SERVO] Joint %d commanded -> Target: %d deg (gliding smoothly)\n", joint, angle);
            AsyncWebServerResponse* res = req->beginResponse(200, "application/json", "{\"status\":\"success\",\"joint\":" + String(joint) + ",\"target\":" + String(angle) + "}");
            addCORS(res);
            req->send(res);
        }
    );

    // Multi-Joint Smooth Coordinated Movement Endpoint
    server.on("/move", HTTP_POST, [](AsyncWebServerRequest* req){}, NULL,
        [](AsyncWebServerRequest* req, uint8_t* data, size_t len, size_t index, size_t total) {
            if (autoMode) {
                AsyncWebServerResponse* res = req->beginResponse(403, "application/json", "{\"error\":\"Auto mode active\"}");
                addCORS(res);
                req->send(res);
                return;
            }

            #if ARDUINOJSON_VERSION_MAJOR >= 7
                JsonDocument doc;
            #else
                StaticJsonDocument<256> doc;
            #endif
            deserializeJson(doc, data, len);

            if (doc.containsKey("base")) targetBase = constrain((int)doc["base"], 0, 180);
            if (doc.containsKey("shoulder")) targetShoulder = constrain((int)doc["shoulder"], 0, 180);
            if (doc.containsKey("elbow")) targetElbow = constrain((int)doc["elbow"], 0, 180);
            if (doc.containsKey("wrist")) targetWrist = constrain((int)doc["wrist"], 0, 180);

            // Optional custom speeds
            if (doc.containsKey("elbowDelay")) elbowDownDelay = constrain((int)doc["elbowDelay"], 15, 120);
            if (doc.containsKey("transitDelay")) {
                baseStepDelay = constrain((int)doc["transitDelay"], 10, 80);
                shoulderStepDelay = baseStepDelay;
            }

            Serial.printf("[SMOOTH MOVE] Targets: Base=%d, Shoulder=%d, Elbow=%d, Wrist=%d\n",
                          targetBase, targetShoulder, targetElbow, targetWrist);

            AsyncWebServerResponse* res = req->beginResponse(200, "application/json", "{\"status\":\"success\",\"message\":\"Smooth motion profile active\"}");
            addCORS(res);
            req->send(res);
        }
    );

    // Toggle Electromagnet
    server.on("/grip", HTTP_POST, [](AsyncWebServerRequest* req){}, NULL,
        [](AsyncWebServerRequest* req, uint8_t* data, size_t len, size_t index, size_t total) {
            #if ARDUINOJSON_VERSION_MAJOR >= 7
                JsonDocument doc;
            #else
                StaticJsonDocument<256> doc;
            #endif
            DeserializationError err = deserializeJson(doc, data, len);
            if (err) {
                AsyncWebServerResponse* res = req->beginResponse(400, "application/json", "{\"error\":\"Invalid JSON\"}");
                addCORS(res);
                req->send(res);
                return;
            }

            String state = doc["state"].as<String>();
            magnetOn = (state == "close" || state == "on" || state == "1" || state == "true");
            digitalWrite(MAGNET_PIN, magnetOn ? HIGH : LOW);

            Serial.printf("[MAGNET] Electromagnet set to %s\n", magnetOn ? "ON" : "OFF");
            AsyncWebServerResponse* res = req->beginResponse(200, "application/json", "{\"status\":\"success\",\"magnetOn\":" + String(magnetOn ? "true" : "false") + "}");
            addCORS(res);
            req->send(res);
        }
    );

    // Toggle Mode (Auto vs Manual)
    server.on("/mode", HTTP_POST, [](AsyncWebServerRequest* req){}, NULL,
        [](AsyncWebServerRequest* req, uint8_t* data, size_t len, size_t index, size_t total) {
            #if ARDUINOJSON_VERSION_MAJOR >= 7
                JsonDocument doc;
            #else
                StaticJsonDocument<256> doc;
            #endif
            DeserializationError err = deserializeJson(doc, data, len);
            if (err) {
                AsyncWebServerResponse* res = req->beginResponse(400, "application/json", "{\"error\":\"Invalid JSON\"}");
                addCORS(res);
                req->send(res);
                return;
            }

            String modeStr = doc["mode"].as<String>();
            autoMode = (modeStr == "auto");

            Serial.printf("[MODE] Mode switched to: %s\n", autoMode ? "AUTO" : "MANUAL");
            AsyncWebServerResponse* res = req->beginResponse(200, "application/json", "{\"status\":\"success\",\"autoMode\":" + String(autoMode ? "true" : "false") + "}");
            addCORS(res);
            req->send(res);
        }
    );

    server.begin();
    Serial.println("[SERVER] HTTP Server started on port 80.");
}

void loop() {
    unsigned long now = millis();

    // 1. Base Servo (Joint 0) - Smooth & steady rotation across workspace and bins
    if (currentBase != targetBase && (now - lastBaseTick >= (unsigned long)baseStepDelay)) {
        lastBaseTick = now;
        if (currentBase < targetBase) currentBase++;
        else currentBase--;
        baseServo.write(currentBase);
        baseAngle = currentBase;
    }

    // 2. Shoulder Servo (Joint 1) - Smooth elevation
    if (currentShoulder != targetShoulder && (now - lastShoulderTick >= (unsigned long)shoulderStepDelay)) {
        lastShoulderTick = now;
        if (currentShoulder < targetShoulder) currentShoulder++;
        else currentShoulder--;
        shoulderServo.write(currentShoulder);
        shoulderAngle = currentShoulder;
    }

    // 3. Elbow Servo (Joint 2) - Slow descent to pick/drop coin, steady ascent
    if (currentElbow != targetElbow) {
        // When moving downward (increasing angle or descending towards coin/bin), move extra slowly
        bool isDescending = (targetElbow > currentElbow); 
        int activeDelay = isDescending ? elbowDownDelay : elbowUpDelay;
        if (now - lastElbowTick >= (unsigned long)activeDelay) {
            lastElbowTick = now;
            if (currentElbow < targetElbow) currentElbow++;
            else currentElbow--;
            elbowServo.write(currentElbow);
            elbowAngle = currentElbow;
        }
    }

    // 4. Wrist Servo (Joint 3) - Steady orientation
    if (currentWrist != targetWrist && (now - lastWristTick >= (unsigned long)wristStepDelay)) {
        lastWristTick = now;
        if (currentWrist < targetWrist) currentWrist++;
        else currentWrist--;
        wristServo.write(currentWrist);
        wristAngle = currentWrist;
    }

    delay(2);
}
