#include <WiFi.h>
#include <WebServer.h>
#include <WiFiUdp.h>
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"

// ==========================================
// WiFi Credentials (same as your network)
// ==========================================
const char* ssid = "Redmi 10A";
const char* password = "BBBBKKKK";

WebServer server(80);
WiFiUDP Udp;
unsigned int localUdpPort = 4210;
char packetBuffer[255];

// =======================================================
// Pinout Configuration for ESP32-DevKitC (38-pin / 30-pin)
// Using standard safe GPIOs (no strapping pins used)
// =======================================================
// L298N Motor Driver Pins (ENA and ENB jumpers kept connected to 5V)
// Direct PWM is applied to IN1-IN4
#define IN1 25  // Left Motor Forward  (PWM)
#define IN2 26  // Left Motor Backward (PWM)
#define IN3 27  // Right Motor Forward  (PWM)
#define IN4 14  // Right Motor Backward (PWM)

// Dual IR Sensor Pins (Active-LOW digital sensors)
#define IR_LEFT  32 // Left IR Sensor (FC-51)
#define IR_RIGHT 33 // Right IR Sensor (FC-51)

// =======================================================
// PWM Configuration for ESP32
// Modern ESP32 Core (v3.x / v2.x) supports ledcAttach
// =======================================================
#define PWM_FREQ 1000    // 1 kHz PWM frequency for DC motors
#define PWM_RES  8       // 8-bit resolution (0 - 255)

unsigned long lastCmdTime = 0;
const unsigned long WATCHDOG_TIMEOUT = 500; // ms

int targetLeftSpeed = 0;
int targetRightSpeed = 0;

void setupMotors() {
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  // Initialize motor pins to LOW
  ledcAttach(IN1, PWM_FREQ, PWM_RES);
  ledcAttach(IN2, PWM_FREQ, PWM_RES);
  ledcAttach(IN3, PWM_FREQ, PWM_RES);
  ledcAttach(IN4, PWM_FREQ, PWM_RES);

  setTargetMotors(0, 0);
}

void setTargetMotors(int leftSpeed, int rightSpeed) {
  targetLeftSpeed = constrain(leftSpeed, -255, 255);
  targetRightSpeed = constrain(rightSpeed, -255, 255);
}

void updateMotors() {
  int lSpeed = targetLeftSpeed;
  int rSpeed = targetRightSpeed;

  // Left Motor (Direct PWM with ENA jumpered to 5V)
  if (lSpeed > 0) {
    ledcWrite(IN1, lSpeed);
    ledcWrite(IN2, 0);
  } else if (lSpeed < 0) {
    ledcWrite(IN1, 0);
    ledcWrite(IN2, -lSpeed);
  } else {
    ledcWrite(IN1, 0);
    ledcWrite(IN2, 0);
  }

  // Right Motor (Direct PWM with ENB jumpered to 5V)
  if (rSpeed > 0) {
    ledcWrite(IN3, rSpeed);
    ledcWrite(IN4, 0);
  } else if (rSpeed < 0) {
    ledcWrite(IN3, 0);
    ledcWrite(IN4, -rSpeed);
  } else {
    ledcWrite(IN3, 0);
    ledcWrite(IN4, 0);
  }
}

// HTTP API: /control?left=X&right=Y
void handleControl() {
  if (server.hasArg("left") && server.hasArg("right")) {
    int left = server.arg("left").toInt();
    int right = server.arg("right").toInt();
    setTargetMotors(left, right);
    lastCmdTime = millis();
    server.send(200, "text/plain", "OK");
  } else {
    server.send(400, "text/plain", "Missing left or right parameter");
  }
}

// HTTP API: /status (Reports rover status; IR sensors disabled)
void handleStatus() {
  // IR sensors disabled/commented out for now (prevents floating pin false triggers)
  // int irLeft = digitalRead(IR_LEFT);
  // int irRight = digitalRead(IR_RIGHT);
  // bool leftBlocked = (irLeft == LOW);
  // bool rightBlocked = (irRight == LOW);

  String json = "{";
  json += "\"left_blocked\":false,";
  json += "\"right_blocked\":false,";
  json += "\"left_speed\":" + String(targetLeftSpeed) + ",";
  json += "\"right_speed\":" + String(targetRightSpeed);
  json += "}";

  server.send(200, "application/json", json);
}

// UDP Packet handler
void handleUdp() {
  int packetSize = Udp.parsePacket();
  if (packetSize) {
    int len = Udp.read(packetBuffer, 255);
    if (len > 0) {
      packetBuffer[len] = '\0';
      char cmd = packetBuffer[0];
      lastCmdTime = millis();
      
      switch (cmd) {
        case 'F': setTargetMotors(200, 200); break;
        case 'B': setTargetMotors(-200, -200); break;
        case 'L': setTargetMotors(-180, 180); break;
        case 'R': setTargetMotors(180, -180); break;
        case 'S': setTargetMotors(0, 0); break;
      }
    }
  }
}

void setup() {
  // Disable brownout detector to prevent reboot loops during high-power WiFi bursts
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);

  Serial.begin(115200);
  delay(1000);
  Serial.println("\n\n========================================================");
  Serial.println("  ESP32 Rover Motor & Dual IR Controller");
  Serial.println("========================================================");

  setupMotors();
  // pinMode(IR_LEFT, INPUT);   // Disabled for now
  // pinMode(IR_RIGHT, INPUT);  // Disabled for now

  // 1. Reset and configure WiFi
  WiFi.persistent(false);
  WiFi.disconnect(true);
  delay(200);
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false); // Disable WiFi sleep to prevent connection drops

  // 2. Scan available networks to check if Redmi 10A is visible
  Serial.println("\n[WiFi] Scanning nearby networks...");
  int n = WiFi.scanNetworks();
  bool foundHotspot = false;
  if (n == 0) {
    Serial.println("[WiFi] No networks found! Check if ESP32 antenna is OK.");
  } else {
    Serial.printf("[WiFi] Found %d networks:\n", n);
    for (int i = 0; i < n; ++i) {
      Serial.printf("  %2d: %-20s (%d dBm, Ch %d) %s\n", 
                    i + 1, 
                    WiFi.SSID(i).c_str(), 
                    WiFi.RSSI(i), 
                    WiFi.channel(i),
                    (WiFi.encryptionType(i) == WIFI_AUTH_OPEN) ? "Open" : "Encrypted");
      if (WiFi.SSID(i) == ssid) {
        foundHotspot = true;
      }
    }
  }

  if (foundHotspot) {
    Serial.printf("\n[WiFi] Target SSID '%s' FOUND in scan! Attempting connection...\n", ssid);
  } else {
    Serial.printf("\n[WiFi] WARNING: Target SSID '%s' NOT seen in scan! Check hotspot name or 2.4GHz band.\n", ssid);
  }

  // 3. Initiate Connection
  Serial.printf("[WiFi] Connecting to '%s' with password '%s'...\n", ssid, password);
  WiFi.begin(ssid, password);

  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 40) {
    delay(500);
    Serial.print(".");
    attempts++;
    if (attempts % 10 == 0) {
      Serial.printf(" [Status code: %d]\n", WiFi.status());
    }
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\n[WiFi] CONNECTED SUCCESSFULLY!");
    Serial.print("[WiFi] IP Address: ");
    Serial.println(WiFi.localIP());
    Serial.print("[WiFi] Gateway: ");
    Serial.println(WiFi.gatewayIP());
    Serial.print("[WiFi] RSSI: ");
    Serial.printf("%d dBm\n", WiFi.RSSI());
  } else {
    Serial.printf("\n[WiFi] FAILED to connect after %d attempts! Final status code: %d\n", attempts, WiFi.status());
    Serial.println("Status reference: 1=NO_SSID_AVAIL, 4=CONNECT_FAILED, 5=CONNECTION_LOST, 6=DISCONNECTED");
    Serial.println("Retrying in background loop...\n");
  }

  server.on("/control", HTTP_GET, handleControl);
  server.on("/status", HTTP_GET, handleStatus);
  server.begin();

  Udp.begin(localUdpPort);
  Serial.printf("UDP listener started on port %d\n", localUdpPort);
}

void loop() {
  server.handleClient();
  handleUdp();
  updateMotors();

  // Auto-reconnect if WiFi drops
  if (WiFi.status() != WL_CONNECTED) {
    WiFi.reconnect();
  }

  // Watchdog - smooth stop if connection lost
  if (millis() - lastCmdTime > WATCHDOG_TIMEOUT && (targetLeftSpeed != 0 || targetRightSpeed != 0)) {
    setTargetMotors(0, 0);
  }

  delay(2);
}
