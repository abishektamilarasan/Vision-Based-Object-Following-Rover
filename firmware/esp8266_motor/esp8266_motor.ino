#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include <WiFiUdp.h>

const char* ssid = "Redmi 10A";
const char* password = "BBBBKKKK";

ESP8266WebServer server(80);
WiFiUDP Udp;
unsigned int localUdpPort = 4210;
char packetBuffer[255];

// Motor Driver Pins (L298N with ENA & ENB jumpers connected to 5V)
// Direct PWM is applied to IN1-IN4. No boot pins (D3/D4/D8) are used!
#define IN1 5   // D1 (Left Motor PWM / Forward)
#define IN2 4   // D2 (Left Motor PWM / Backward)

#define IN3 14  // D5 (Right Motor PWM / Forward)
#define IN4 12  // D6 (Right Motor PWM / Backward)

// Dual IR Sensor Pins (Active-LOW digital sensors)
#define IR_LEFT  13 // D7 (Boot-safe)
#define IR_RIGHT 16 // D0 (Boot-safe)

unsigned long lastCmdTime = 0;
const unsigned long WATCHDOG_TIMEOUT = 500; // ms

int targetLeftSpeed = 0;
int targetRightSpeed = 0;
float currentLeftSpeed = 0;
float currentRightSpeed = 0;
const float RAMP_STEP = 10.0; // Speed change per loop iteration

void setupMotors() {
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  analogWriteRange(255); // Standard 8-bit PWM (0-255)
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
    analogWrite(IN1, lSpeed);
    analogWrite(IN2, 0);
  } else if (lSpeed < 0) {
    analogWrite(IN1, 0);
    analogWrite(IN2, -lSpeed);
  } else {
    analogWrite(IN1, 0);
    analogWrite(IN2, 0);
  }

  // Right Motor (Direct PWM with ENB jumpered to 5V)
  if (rSpeed > 0) {
    analogWrite(IN3, rSpeed);
    analogWrite(IN4, 0);
  } else if (rSpeed < 0) {
    analogWrite(IN3, 0);
    analogWrite(IN4, -rSpeed);
  } else {
    analogWrite(IN3, 0);
    analogWrite(IN4, 0);
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

// HTTP API: /status (Reports dual IR sensor states)
void handleStatus() {
  // Read IR sensors (LOW means obstacle detected for active-low IR modules)
  int irLeft = digitalRead(IR_LEFT);
  int irRight = digitalRead(IR_RIGHT);
  
  bool leftBlocked = (irLeft == LOW);
  bool rightBlocked = (irRight == LOW);

  String json = "{";
  json += "\"left_blocked\":" + String(leftBlocked ? "true" : "false") + ",";
  json += "\"right_blocked\":" + String(rightBlocked ? "true" : "false") + ",";
  json += "\"left_speed\":" + String((int)currentLeftSpeed) + ",";
  json += "\"right_speed\":" + String((int)currentRightSpeed);
  json += "}";

  server.send(200, "application/json", json);
}

// UDP Packet handler (from CS_ArUco_Imp prototype)
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
  Serial.begin(115200); 
  Serial.println("\nStarting ESP8266 Rover Motor & Dual IR Controller...");
  
  setupMotors();
  pinMode(IR_LEFT, INPUT);
  pinMode(IR_RIGHT, INPUT);

  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi..");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.println("Connected to WiFi");
  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP());

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

  // Watchdog - smooth stop if connection lost
  if (millis() - lastCmdTime > WATCHDOG_TIMEOUT && (targetLeftSpeed != 0 || targetRightSpeed != 0)) {
    setTargetMotors(0, 0);
  }
  
  delay(10);
}
