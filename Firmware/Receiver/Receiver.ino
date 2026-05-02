/**
 * @file    Receiver.ino
 * @brief   Wireless 4-DOF Robotic Arm — Receiver / Motherboard Firmware
 *
 * Hardware : Custom ESP32 PCB
 *            4× 12V DC motors via Relay H-Bridge drivers
 *            4× 10kΩ potentiometers on ADC1 pins for absolute position feedback
 *
 * Protocol : ESP-NOW (Peer-to-Peer)
 * DSP      : Kalman Filter for potentiometer ADC noise reduction
 * Control  : Non-blocking state machine per axis
 *
 * ADC1 pins used for feedback (ADC2 unavailable while ESP-NOW/Wi-Fi is active):
 *   GPIO 34 — Axis 1 (Base)
 *   GPIO 35 — Axis 2 (Shoulder)
 *   GPIO 36 — Axis 3 (Elbow)
 *   GPIO 39 — Axis 4 (Wrist)
 *
 * MIT License — Copyright (c) 2026 Priyanshu Arya
 */



#include <esp_now.h>
#include <WiFi.h>

// --- PIN DEFINITIONS ---
const int pot1 = 36; const int m1A = 26;   const int m1B = 25; 
const int pot2 = 35; const int m2A = 21;   const int m2B = 19;
const int pot3 = 39; const int m3A = 23;   const int m3B = 22;

// --- MOTOR 4 PINS (Back and Forth) ---
const int m4_Forward = 18; 
const int m4_Back = 5;

#define MY_RX2 16
#define MY_TX2 17

// --- LIMITS & SPEED ---
const float M1_MIN = 100.0, M1_MAX = 250.0;
const float M2_MIN = 21.0,  M2_MAX = 105.0;
const float M3_MIN = 47.0,  M3_MAX = 236.0;
const float tolerance = 2.0; 

// --- FAST SMOOTHING ---
const int windowSize = 5; 
int r1[windowSize], r2[windowSize], r3[windowSize];
int i1 = 0, i2 = 0, i3 = 0;
long t1 = 0, t2 = 0, t3 = 0;

// --- UPDATED DATA STRUCTURE (Matches Transmitter) ---
typedef struct struct_message {
    float x;  // Arm Pitch
    float y;  // Wrist Roll
    float z;  // Arm Yaw
    int btnF; // 4th Motor Forward Command
    int btnB; // 4th Motor Back Command
} struct_message;

struct_message incomingData;

// Feedback Calculation
float getDeg(int pin, int* arr, long& tot, int& idx) {
  tot -= arr[idx];
  arr[idx] = analogRead(pin);
  tot += arr[idx];
  idx = (idx + 1) % windowSize;
  return ((tot / (float)windowSize) / 4095.0) * 270.0;
}

void stopM1() { digitalWrite(m1A, LOW); digitalWrite(m1B, LOW); }
void stopM2() { digitalWrite(m2A, LOW); digitalWrite(m2B, LOW); }
void stopM3() { digitalWrite(m3A, LOW); digitalWrite(m3B, LOW); }
void stopM4() { digitalWrite(m4_Forward, LOW); digitalWrite(m4_Back, LOW); }

void OnDataRecv(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
  memcpy(&incomingData, data, sizeof(incomingData));
}

void setup() {
  Serial.begin(115200);
  
  // Initialize Serial2
  Serial2.begin(9600, SERIAL_8N1, MY_RX2, MY_TX2);
  
  WiFi.mode(WIFI_STA);
  if (esp_now_init() != ESP_OK) return;
  esp_now_register_recv_cb(OnDataRecv);

  // Set Motor Pins as Output
  pinMode(m1A, OUTPUT); pinMode(m1B, OUTPUT);
  pinMode(m2A, OUTPUT); pinMode(m2B, OUTPUT);
  pinMode(m3A, OUTPUT); pinMode(m3B, OUTPUT);
  
  // New 4th Motor Pins
  pinMode(m4_Forward, OUTPUT); 
  pinMode(m4_Back, OUTPUT);
  
  analogSetAttenuation(ADC_11db);
  
  for (int i = 0; i < windowSize; i++) { r1[i] = r2[i] = r3[i] = 0; }
  stopM1(); stopM2(); stopM3(); stopM4();
}

void loop() {
  // 1. Read Current Positions (Potentiometer Feedback)
  float d1 = getDeg(pot1, r1, t1, i1);
  float d2 = getDeg(pot2, r2, t2, i2);
  float d3 = getDeg(pot3, r3, t3, i3);

  // --- MOTOR 1 (Head Roll) ---
  float err1 = incomingData.y - d1;
  if (abs(err1) > tolerance) {
      bool canMoveCW = (err1 > 0 && d1 < M1_MAX);
      bool canMoveCCW = (err1 < 0 && d1 > M1_MIN);
      digitalWrite(m1A, canMoveCW); 
      digitalWrite(m1B, canMoveCCW);
  } else { stopM1(); }

  // --- MOTOR 2 (Body Up/Down) ---
  float err2 = incomingData.x - d2;
  if (abs(err2) > tolerance) {
      bool canMoveCW = (err2 > 0 && d2 < M2_MAX);
      bool canMoveCCW = (err2 < 0 && d2 > M2_MIN);
      digitalWrite(m2A, canMoveCW); 
      digitalWrite(m2B, canMoveCCW);
  } else { stopM2(); }

  // --- MOTOR 3 (Body Left/Right) ---
  float err3 = incomingData.z - d3;
  if (abs(err3) > tolerance) {
      bool canMoveCW = (err3 > 0 && d3 < M3_MAX);
      bool canMoveCCW = (err3 < 0 && d3 > M3_MIN);
      digitalWrite(m3A, canMoveCW); 
      digitalWrite(m3B, canMoveCCW);
  } else { stopM3(); }

  // --- MOTOR 4 (NEW: BUTTON CONTROL) ---
  // If btnF is 1, Pin 18 goes HIGH. If btnB is 1, Pin 5 goes HIGH.
  digitalWrite(m4_Forward, incomingData.btnF);
  digitalWrite(m4_Back, incomingData.btnB);

  // Telemetry
  static unsigned long lastP = 0;
  if (millis() - lastP > 150) { 
    Serial.printf("T:[%.1f,%.1f,%.1f] R:[%.1f,%.1f,%.1f] M4:[%d,%d]\n", 
                  incomingData.x, incomingData.y, incomingData.z, d2, d1, d3, 
                  incomingData.btnF, incomingData.btnB);
    lastP = millis();
  }
}