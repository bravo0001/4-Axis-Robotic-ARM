/**
 * @file    Transmitter.ino
 * @brief   Wireless 4-DOF Robotic Arm — Transmitter Firmware
 *
 * Hardware : ESP32 DevKit + MPU-6050 IMU (I²C)
 * Protocol : ESP-NOW (Peer-to-Peer, no router required)
 * DSP      : Kalman Filter for accelerometer / gyroscope fusion
 *
 * Reads 4-axis orientation from the IMU, filters with a Kalman filter,
 * and broadcasts the result to the Receiver via ESP-NOW ~every 20 ms.
 *
 * Steps to use:
 *  1. Upload this sketch to the Transmitter ESP32.
 *  2. Set `receiverMacAddress` to the MAC address of your Receiver ESP32.
 *     (Run `WiFi.macAddress()` on the Receiver to find it.)
 *
 * MIT License — Copyright (c) 2026 Priyanshu Arya
 */

#include <esp_now.h>
#include <WiFi.h>
#include <Wire.h>

// --- BUTTON PINS FOR 4th MOTOR ---
#define BTN_F 14  // Move 4th Motor Forward
#define BTN_B 12  // Move 4th Motor Back

// --- RECEIVER MAC ADDRESS ---
uint8_t broadcastAddress[] = {0xD, 0xE9, 0xF4, 0x69, 0x37, 0x34}; 

const int MPU_addr = 0x68; 

class HeavyKalman {
  public:
    float Q_angle = 0.001f, Q_bias = 0.003f, R_measure = 0.03f;
    float angle = 0, bias = 0;  
    float P[2][2] = {{0, 0}, {0, 0}};

    float getAngle(float newAngle, float newRate, float dt) {
      float rate = newRate - bias;
      angle += dt * rate;
      P[0][0] += dt * (dt * P[1][1] - P[0][1] - P[1][0] + Q_angle);
      P[0][1] -= dt * P[1][1];
      P[1][0] -= dt * P[1][1];
      P[1][1] += Q_bias * dt;
      float S = P[0][0] + R_measure;
      float K[2] = {P[0][0] / S, P[1][0] / S};
      float y = newAngle - angle;
      angle += K[0] * y;
      bias += K[1] * y;
      float P00_temp = P[0][0], P01_temp = P[0][1];
      P[0][0] -= K[0] * P00_temp; P[0][1] -= K[0] * P01_temp;
      P[1][0] -= K[1] * P00_temp; P[1][1] -= K[1] * P01_temp;
      return angle;
    };
};

struct SensorData {
  float home_pitch = 0;
  float gyroX_offset = 0;
  float gyroZ_offset = 0;
  float z_angle = 0;      
  HeavyKalman kalmanX;
};

// --- UPDATED DATA STRUCTURE ---
typedef struct struct_message {
    float x; // Arm Pitch
    float y; // Wrist Roll
    float z; // Arm Yaw
    int btnF; // 4th Motor Forward (1 or 0)
    int btnB; // 4th Motor Back (1 or 0)
} struct_message;

struct_message myData;
esp_now_peer_info_t peerInfo;
SensorData mpu1, mpu2;
unsigned long timer;

void OnDataSent(const wifi_tx_info_t *tx_info, esp_now_send_status_t status) {}

int16_t readRaw(TwoWire &bus, int reg) {
    bus.beginTransmission(0x68);
    bus.write(reg);
    if (bus.endTransmission(false) != 0) return -999; 
    bus.requestFrom(0x68, 2);
    if (bus.available() < 2) return -999;
    return (bus.read() << 8 | bus.read());
}

void setupSensor(TwoWire &bus) {
  bus.beginTransmission(MPU_addr);
  bus.write(0x6B); bus.write(0);    
  bus.endTransmission();
}

void processMPU(TwoWire &bus, SensorData &data, float dt, bool onlyX) {
  int16_t ax = readRaw(bus, 0x3B);
  int16_t ay = readRaw(bus, 0x3D);
  int16_t az = readRaw(bus, 0x3F);
  int16_t gx = readRaw(bus, 0x43);
  int16_t gz = readRaw(bus, 0x47);

  if(ax == -999) return; 

  float accX = (atan2(-ax, sqrt((long)ay * ay + (long)az * az)) * 180 / M_PI) - data.home_pitch;
  float gyroXrate = (gx - data.gyroX_offset) / 131.0;
  data.kalmanX.getAngle(accX, gyroXrate, dt);

  if (!onlyX) {
    float gyroZrate = (gz - data.gyroZ_offset) / 131.0;
    if (abs(gyroZrate) > 0.8) data.z_angle += gyroZrate * dt;
  }
}

void calibrateAll(TwoWire &bus, SensorData &data) {
  float sumP = 0, sumGX = 0, sumGZ = 0;
  for (int i = 0; i < 200; i++) {
    int16_t ax = readRaw(bus, 0x3B);
    int16_t ay = readRaw(bus, 0x3D);
    int16_t az = readRaw(bus, 0x3F);
    sumP += atan2(-ax, sqrt((long)ay * ay + (long)az * az)) * 180 / M_PI;
    sumGX += readRaw(bus, 0x43);
    sumGZ += readRaw(bus, 0x47);
    delay(2);
  }
  data.home_pitch = sumP / 200.0;
  data.gyroX_offset = sumGX / 200.0;
  data.gyroZ_offset = sumGZ / 200.0;
}

void setup() {
  Serial.begin(115200);
  
  // Initialize Buttons with Pullups
  pinMode(BTN_F, INPUT_PULLUP);
  pinMode(BTN_B, INPUT_PULLUP);

  WiFi.mode(WIFI_STA);
  esp_now_init();
  esp_now_register_send_cb(OnDataSent);
  
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;
  esp_now_add_peer(&peerInfo);

  Wire.begin(21, 22);   
  Wire1.begin(33, 32);  

  setupSensor(Wire);
  setupSensor(Wire1);

  Serial.println("HEAVY CALIBRATION...");
  calibrateAll(Wire, mpu1);
  calibrateAll(Wire1, mpu2);
  
  timer = micros();
}

void loop() {
    float dt = (float)(micros() - timer) / 1000000.0;
    timer = micros();

    processMPU(Wire, mpu1, dt, true); 
    
    int16_t testRead = readRaw(Wire1, 0x3B);
    if (testRead == -999) {
        Serial.println("!!! MPU 2 (ARM) DISCONNECTED !!!");
        Wire1.begin(33, 32); 
    } else {
        processMPU(Wire1, mpu2, dt, false);
    }

    // --- ANALOG MAPPING ---
    myData.x = constrain(map(mpu2.kalmanX.angle, -45, 45, 21, 105), 21, 105);
    myData.y = constrain(map(mpu1.kalmanX.angle, -35, 35, 110, 250), 110, 250);
    myData.z = constrain(map(mpu2.z_angle, -45, 45, 236, 47), 47, 236);

    // --- NEW BUTTON LOGIC ---
    // Read pins. Low = Pressed (because of INPUT_PULLUP)
    myData.btnF = (digitalRead(BTN_F) == LOW) ? 1 : 0;
    myData.btnB = (digitalRead(BTN_B) == LOW) ? 1 : 0;

    esp_now_send(broadcastAddress, (uint8_t *) &myData, sizeof(myData));
     
    Serial.printf("TX -> P:%.1f R:%.1f Y:%.1f | B4:%d,%d\n", 
                  myData.x, myData.y, myData.z, myData.btnF, myData.btnB);
    delay(20); 
}