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

// Hardware State
int baseAngle     = 90;
int shoulderAngle = 90;
int elbowAngle    = 90;
int wristAngle    = 90;
bool magnetOn     = false;
bool autoMode     = false;

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
            StaticJsonDocument<256> doc;
        #endif
        doc["status"] = "ok";
        doc["base"] = baseAngle;
        doc["shoulder"] = shoulderAngle;
        doc["elbow"] = elbowAngle;
        doc["wrist"] = wristAngle;
        doc["magnet"] = magnetOn;
        doc["autoMode"] = autoMode;

        String response;
        serializeJson(doc, response);
        AsyncWebServerResponse* res = req->beginResponse(200, "application/json", response);
        addCORS(res);
        req->send(res);
    });

    // Set Joint Angle (0=Base, 1=Shoulder, 2=Elbow, 3=Wrist)
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

            switch (joint) {
                case 0:
                    baseAngle = angle;
                    baseServo.write(angle);
                    break;
                case 1:
                    shoulderAngle = angle;
                    shoulderServo.write(angle);
                    break;
                case 2:
                    elbowAngle = angle;
                    elbowServo.write(angle);
                    break;
                case 3:
                    wristAngle = angle;
                    wristServo.write(angle);
                    break;
                default:
                    AsyncWebServerResponse* res = req->beginResponse(400, "application/json", "{\"error\":\"Unknown joint index\"}");
                    addCORS(res);
                    req->send(res);
                    return;
            }

            Serial.printf("[SERVO] Joint %d set to %d deg\n", joint, angle);
            AsyncWebServerResponse* res = req->beginResponse(200, "application/json", "{\"status\":\"success\"}");
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
    // AsyncWebServer handles requests asynchronously on ESP32 background tasks
    delay(100);
}
