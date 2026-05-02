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

#include <WiFi.h>
#include <esp_now.h>
#include <Wire.h>

// ---------------------------------------------------------------------------
// Configuration
// ---------------------------------------------------------------------------

/** MAC address of the Receiver ESP32 — update before flashing. */
static uint8_t receiverMacAddress[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

/** Transmission interval in milliseconds. */
static const uint32_t TX_INTERVAL_MS = 20;

// MPU-6050 I²C address (AD0 pin low = 0x68, high = 0x69)
static const uint8_t MPU6050_ADDR     = 0x68;
static const uint8_t MPU6050_PWR_MGMT = 0x6B;
static const uint8_t MPU6050_ACCEL_XOUT_H = 0x3B;
static const uint8_t MPU6050_GYRO_XOUT_H  = 0x43;

// ---------------------------------------------------------------------------
// Data packet — must match the struct in Receiver.ino
// ---------------------------------------------------------------------------
typedef struct {
  float axis1_angle;  // Base     (degrees)
  float axis2_angle;  // Shoulder (degrees)
  float axis3_angle;  // Elbow    (degrees)
  float axis4_angle;  // Wrist    (degrees)
} RoboArmPacket;

static RoboArmPacket txPacket;

// ---------------------------------------------------------------------------
// Kalman Filter (single-variable, discrete)
// ---------------------------------------------------------------------------
struct KalmanFilter {
  float Q;        ///< Process noise covariance (tune to motor vibration)
  float R;        ///< Measurement noise covariance (tune to sensor noise)
  float P;        ///< Estimate error covariance
  float K;        ///< Kalman gain
  float estimate; ///< Current state estimate

  KalmanFilter(float q = 0.001f, float r = 0.03f)
      : Q(q), R(r), P(1.0f), K(0.0f), estimate(0.0f) {}

  float update(float measurement) {
    // Prediction step
    P += Q;
    // Update step
    K = P / (P + R);
    estimate += K * (measurement - estimate);
    P *= (1.0f - K);
    return estimate;
  }
};

static KalmanFilter kfAxis1;
static KalmanFilter kfAxis2;
static KalmanFilter kfAxis3;
static KalmanFilter kfAxis4;

// ---------------------------------------------------------------------------
// Raw IMU data
// ---------------------------------------------------------------------------
struct ImuRaw {
  int16_t ax, ay, az;
  int16_t gx, gy, gz;
};

static ImuRaw imuRaw;
static float pitchDeg = 0.0f;
static float rollDeg  = 0.0f;

// Complementary filter weight (0 = pure accel, 1 = pure gyro integration)
static const float ALPHA = 0.96f;
static uint32_t lastImuUs = 0;

// ---------------------------------------------------------------------------
// ESP-NOW send callback
// ---------------------------------------------------------------------------
static void onDataSent(const uint8_t *macAddr, esp_now_send_status_t status) {
  // Optional: log delivery status during development
  // Serial.print("TX status: ");
  // Serial.println(status == ESP_NOW_SEND_SUCCESS ? "OK" : "FAIL");
}

// ---------------------------------------------------------------------------
// MPU-6050 helpers
// ---------------------------------------------------------------------------
static void mpu6050Init() {
  Wire.begin();
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(MPU6050_PWR_MGMT);
  Wire.write(0x00); // Wake up
  Wire.endTransmission(true);
}

static bool mpu6050Read(ImuRaw &out) {
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(MPU6050_ACCEL_XOUT_H);
  if (Wire.endTransmission(false) != 0) return false;

  Wire.requestFrom((uint8_t)MPU6050_ADDR, (uint8_t)14, (uint8_t) true);
  if (Wire.available() < 14) return false;

  out.ax = (Wire.read() << 8) | Wire.read();
  out.ay = (Wire.read() << 8) | Wire.read();
  out.az = (Wire.read() << 8) | Wire.read();
  Wire.read(); Wire.read(); // Temperature (unused)
  out.gx = (Wire.read() << 8) | Wire.read();
  out.gy = (Wire.read() << 8) | Wire.read();
  out.gz = (Wire.read() << 8) | Wire.read();
  return true;
}

// ---------------------------------------------------------------------------
// setup()
// ---------------------------------------------------------------------------
void setup() {
  Serial.begin(115200);
  Serial.println("[Transmitter] Booting...");

  // --- IMU ---
  mpu6050Init();
  lastImuUs = micros();

  // --- Wi-Fi in STA mode (required for ESP-NOW) ---
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();

  // --- ESP-NOW ---
  if (esp_now_init() != ESP_OK) {
    Serial.println("[Transmitter] ESP-NOW init FAILED — halting.");
    while (true) { delay(1000); }
  }
  esp_now_register_send_cb(onDataSent);

  // Register receiver peer
  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, receiverMacAddress, 6);
  peerInfo.channel = 0;  // Use current channel
  peerInfo.encrypt = false;

  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("[Transmitter] Failed to add peer — check MAC address.");
    while (true) { delay(1000); }
  }

  Serial.println("[Transmitter] Ready. Sending packets every " + String(TX_INTERVAL_MS) + " ms.");
}

// ---------------------------------------------------------------------------
// loop()
// ---------------------------------------------------------------------------
void loop() {
  static uint32_t lastTxMs = 0;

  // Read & fuse IMU at full speed for accurate angle integration
  if (mpu6050Read(imuRaw)) {
    uint32_t nowUs = micros();
    float dt = (nowUs - lastImuUs) * 1e-6f;
    lastImuUs = nowUs;

    // Accelerometer-based angles (degrees)
    float accelPitch = atan2f((float)imuRaw.ay,
                              sqrtf((float)imuRaw.ax * imuRaw.ax +
                                    (float)imuRaw.az * imuRaw.az)) * (180.0f / M_PI);
    float accelRoll  = atan2f(-(float)imuRaw.ax, (float)imuRaw.az) * (180.0f / M_PI);

    // Gyro-based angular rate (degrees/s) — MPU-6050 default ±250 °/s = 131 LSB/°/s
    float gyroPitchRate = (float)imuRaw.gy / 131.0f;
    float gyroRollRate  = (float)imuRaw.gx / 131.0f;
    float gyroYawRate   = (float)imuRaw.gz / 131.0f;

    // Complementary filter: blend gyro integration with accel correction
    pitchDeg = ALPHA * (pitchDeg + gyroPitchRate * dt) + (1.0f - ALPHA) * accelPitch;
    rollDeg  = ALPHA * (rollDeg  + gyroRollRate  * dt) + (1.0f - ALPHA) * accelRoll;

    // Kalman filter on the fused angles and raw yaw rate for 4 axes
    txPacket.axis1_angle = kfAxis1.update(pitchDeg);
    txPacket.axis2_angle = kfAxis2.update(rollDeg);
    txPacket.axis3_angle = kfAxis3.update(pitchDeg + rollDeg);  // Combined proxy
    txPacket.axis4_angle = kfAxis4.update(gyroYawRate * dt);    // Incremental yaw
  }

  // Transmit at the configured interval
  uint32_t nowMs = millis();
  if (nowMs - lastTxMs >= TX_INTERVAL_MS) {
    lastTxMs = nowMs;
    esp_err_t result = esp_now_send(receiverMacAddress,
                                    (uint8_t *)&txPacket,
                                    sizeof(txPacket));
    if (result != ESP_OK) {
      Serial.println("[Transmitter] Send error: " + String(esp_err_to_name(result)));
    }
  }
}
