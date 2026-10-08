#include <WiFi.h>
#include <WebSocketsServer.h>
#include <Wire.h>
#include <SparkFun_VL53L5CX_Library.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define SDA_PIN 8
#define SCL_PIN 9
#define OLED_ADDR 0x3C
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64

const char* ssid = "3D_Scanner_AP";
const char* password = "scanner123";

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);
SparkFun_VL53L5CX myImager;
VL53L5CX_ResultsData measurementData;
WebSocketsServer webSocket = WebSocketsServer(81);

uint8_t mpuAddr = 0x68;
bool mpuFound = false;
bool sensorFound = false;
unsigned long totalFrames = 0;
bool clientConnected = false;
float pitch = 0.0, roll = 0.0;
unsigned long lastOledTime = 0;

char jsonBuffer[700];

void writeRegister(uint8_t address, uint8_t reg, uint8_t val) {
  Wire.beginTransmission(address);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission();
}

int16_t read16Bit(uint8_t address, uint8_t reg) {
  Wire.beginTransmission(address);
  Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom(address, (uint8_t)2);
  if (Wire.available() >= 2) {
    return (Wire.read() << 8) | Wire.read();
  }
  return 0;
}

void updateIMU() {
  if (!mpuFound) return;
  int16_t ax = read16Bit(mpuAddr, 0x3B);
  int16_t ay = read16Bit(mpuAddr, 0x3D);
  int16_t az = read16Bit(mpuAddr, 0x3F);

  float accX = ax / 16384.0;
  float accY = ay / 16384.0;
  float accZ = az / 16384.0;

  float denominator = sqrt(accY * accY + accZ * accZ);
  if (denominator > 0.001) {
    pitch = atan2(-accX, denominator) * 180.0 / M_PI;
  }
  if (abs(accZ) > 0.001 || abs(accY) > 0.001) {
    roll = atan2(accY, accZ) * 180.0 / M_PI;
  }
}

void renderOLED() {
  display.clearDisplay();
  display.setCursor(0, 0);
  display.print("AP: "); display.println(ssid);
  display.setCursor(0, 10);
  display.print("IP: "); display.println(WiFi.softAPIP());
  display.setCursor(0, 20);
  display.print("ToF: "); display.println(sensorFound ? "SENSOR OK" : "NOT FOUND!");
  display.setCursor(0, 30);
  display.print("Client: "); display.println(clientConnected ? "CONNECTED" : "WAITING...");
  display.setCursor(0, 42);
  display.print("P:"); display.print((int)pitch);
  display.print(" R:"); display.print((int)roll);
  display.print(" F:"); display.print(totalFrames);
  display.display();
}

void webSocketEvent(uint8_t num, WStype_t type, uint8_t * payload, size_t length) {
  switch(type) {
    case WStype_DISCONNECTED:
      clientConnected = false;
      Serial.printf("[%u] Disconnected\n", num);
      renderOLED();
      break;
    case WStype_CONNECTED:
      for (uint8_t i = 0; i < 5; i++) {
        if (i != num) webSocket.disconnect(i);
      }
      clientConnected = true;
      Serial.printf("[%u] Connected!\n", num);
      renderOLED();
      break;
  }
}

void sendSensorData() {
  int offset = snprintf(jsonBuffer, sizeof(jsonBuffer), "{\"pitch\":%.1f,\"roll\":%.1f,\"grid\":[", pitch, roll);
  for (int i = 0; i < 64; i++) {
    int dist = measurementData.distance_mm[i];
    if (dist < 0) dist = 0;
    offset += snprintf(jsonBuffer + offset, sizeof(jsonBuffer) - offset, "%d%s", dist, (i < 63) ? "," : "");
  }
  snprintf(jsonBuffer + offset, sizeof(jsonBuffer) - offset, "]}");
  webSocket.broadcastTXT(jsonBuffer);
}

void setup() {
  Serial.begin(115200);
  delay(1000);

  // 400 kHz I2C Fast Mode
  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(400000);

  if (display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR)) {
    display.clearDisplay();
    display.setTextColor(SSD1306_WHITE);
    display.setTextSize(1);
    display.setCursor(0, 0);
    display.println("Booting...");
    display.display();
  }

  // MPU6050 Init
  Wire.beginTransmission(0x68);
  if (Wire.endTransmission() == 0) {
    mpuAddr = 0x68;
    mpuFound = true;
  } else {
    Wire.beginTransmission(0x69);
    if (Wire.endTransmission() == 0) {
      mpuAddr = 0x69;
      mpuFound = true;
    }
  }

  if (mpuFound) {
    writeRegister(mpuAddr, 0x6B, 0x00);
    Serial.printf("[HARDWARE] MPU6050 at 0x%02X\n", mpuAddr);
  }

  // VL53L5CX Init
  if (myImager.begin()) {
    sensorFound = true;
    myImager.setResolution(8 * 8);
    myImager.setRangingFrequency(15);
    myImager.startRanging();
    delay(200);
    Serial.println("[HARDWARE] VL53L5CX Ready!");
  } else {
    Serial.println("[ERROR] VL53L5CX FAILED!");
  }

  WiFi.softAP(ssid, password);
  webSocket.begin();
  webSocket.onEvent(webSocketEvent);

  renderOLED();
}

void loop() {
  webSocket.loop();
  updateIMU();

  if (sensorFound && myImager.isDataReady()) {
    if (myImager.getRangingData(&measurementData)) {
      totalFrames++;
      if (clientConnected) {
        sendSensorData();
      }
    }
  }

  // Refresh OLED once per second to avoid clogging I2C bus
  if (millis() - lastOledTime > 1000) {
    lastOledTime = millis();
    renderOLED();
  }

  yield();
}