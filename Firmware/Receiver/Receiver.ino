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

#include <WiFi.h>
#include <esp_now.h>

// ---------------------------------------------------------------------------
// Pin Definitions
// ---------------------------------------------------------------------------

// Potentiometer feedback (ADC1 — safe with ESP-NOW active)
static const uint8_t PIN_POT_AXIS1 = 34;
static const uint8_t PIN_POT_AXIS2 = 35;
static const uint8_t PIN_POT_AXIS3 = 36;
static const uint8_t PIN_POT_AXIS4 = 39;

// H-Bridge relay control pins — one pair (FWD / REV) per axis.
// Adjust to match your PCB layout.
static const uint8_t PIN_M1_FWD = 16;  static const uint8_t PIN_M1_REV = 17;
static const uint8_t PIN_M2_FWD = 18;  static const uint8_t PIN_M2_REV = 19;
static const uint8_t PIN_M3_FWD = 21;  static const uint8_t PIN_M3_REV = 22;
static const uint8_t PIN_M4_FWD = 23;  static const uint8_t PIN_M4_REV = 25;

// ---------------------------------------------------------------------------
// Configuration
// ---------------------------------------------------------------------------

/** ADC 12-bit full-scale maps to this angle (degrees). Adjust for pot range. */
static const float ADC_MAX_ANGLE_DEG = 270.0f;

/** Dead-band: stop motor when within ±DEADBAND_DEG of target. */
static const float DEADBAND_DEG = 2.0f;

/** Safety timeout: if an axis doesn't reach target within this time, stop. */
static const uint32_t AXIS_TIMEOUT_MS = 5000;

// ---------------------------------------------------------------------------
// Data packet — must match the struct in Transmitter.ino
// ---------------------------------------------------------------------------
typedef struct {
  float axis1_angle;
  float axis2_angle;
  float axis3_angle;
  float axis4_angle;
} RoboArmPacket;

// Latest received packet (written by ESP-NOW ISR, read by loop)
static volatile bool     newDataAvailable = false;
static RoboArmPacket     latestPacket     = {0, 0, 0, 0};
static portMUX_TYPE      packetMux        = portMUX_INITIALIZER_UNLOCKED;

// ---------------------------------------------------------------------------
// Kalman Filter (single-variable, discrete)
// ---------------------------------------------------------------------------
struct KalmanFilter {
  float Q;
  float R;
  float P;
  float K;
  float estimate;

  KalmanFilter(float q = 0.001f, float r = 0.1f)
      : Q(q), R(r), P(1.0f), K(0.0f), estimate(0.0f) {}

  float update(float measurement) {
    P += Q;
    K  = P / (P + R);
    estimate += K * (measurement - estimate);
    P *= (1.0f - K);
    return estimate;
  }
};

static KalmanFilter kfPot1;
static KalmanFilter kfPot2;
static KalmanFilter kfPot3;
static KalmanFilter kfPot4;

// ---------------------------------------------------------------------------
// Motor helpers
// ---------------------------------------------------------------------------

struct MotorPins {
  uint8_t fwd;
  uint8_t rev;
};

static const MotorPins motors[4] = {
  {PIN_M1_FWD, PIN_M1_REV},
  {PIN_M2_FWD, PIN_M2_REV},
  {PIN_M3_FWD, PIN_M3_REV},
  {PIN_M4_FWD, PIN_M4_REV},
};

static void motorStop(uint8_t axis) {
  digitalWrite(motors[axis].fwd, LOW);
  digitalWrite(motors[axis].rev, LOW);
}

static void motorForward(uint8_t axis) {
  digitalWrite(motors[axis].rev, LOW);
  digitalWrite(motors[axis].fwd, HIGH);
}

static void motorReverse(uint8_t axis) {
  digitalWrite(motors[axis].fwd, LOW);
  digitalWrite(motors[axis].rev, HIGH);
}

// ---------------------------------------------------------------------------
// Non-blocking state machine per axis
// ---------------------------------------------------------------------------
enum class AxisState : uint8_t {
  IDLE,      ///< At target, motor stopped
  MOVING_FWD,///< Motor running forward (increasing angle)
  MOVING_REV,///< Motor running in reverse (decreasing angle)
  TIMEOUT    ///< Motion timed-out; motor stopped, awaiting new target
};

struct AxisController {
  uint8_t     potPin;
  KalmanFilter *kf;
  float       targetDeg   = 0.0f;
  AxisState   state       = AxisState::IDLE;
  uint32_t    motionStartMs = 0;

  /** Read potentiometer and return Kalman-filtered angle in degrees. */
  float readAngleDeg() {
    int raw = analogRead(potPin);
    float rawDeg = (raw / 4095.0f) * ADC_MAX_ANGLE_DEG;
    return kf->update(rawDeg);
  }

  /** Call this every loop iteration — drives motor toward target non-blocking. */
  void update(uint8_t axisIndex) {
    float currentDeg = readAngleDeg();
    float error      = targetDeg - currentDeg;

    switch (state) {
      case AxisState::IDLE:
        if (fabsf(error) > DEADBAND_DEG) {
          state = (error > 0) ? AxisState::MOVING_FWD : AxisState::MOVING_REV;
          motionStartMs = millis();
          (error > 0) ? motorForward(axisIndex) : motorReverse(axisIndex);
        }
        break;

      case AxisState::MOVING_FWD:
        if (error <= DEADBAND_DEG) {
          motorStop(axisIndex);
          state = AxisState::IDLE;
        } else if (millis() - motionStartMs > AXIS_TIMEOUT_MS) {
          motorStop(axisIndex);
          state = AxisState::TIMEOUT;
          Serial.printf("[Receiver] Axis %u TIMEOUT (fwd)\n", axisIndex + 1);
        }
        break;

      case AxisState::MOVING_REV:
        if (error >= -DEADBAND_DEG) {
          motorStop(axisIndex);
          state = AxisState::IDLE;
        } else if (millis() - motionStartMs > AXIS_TIMEOUT_MS) {
          motorStop(axisIndex);
          state = AxisState::TIMEOUT;
          Serial.printf("[Receiver] Axis %u TIMEOUT (rev)\n", axisIndex + 1);
        }
        break;

      case AxisState::TIMEOUT:
        // Stay stopped until a new target arrives (handled in loop on newData)
        break;
    }
  }

  void setTarget(float newTargetDeg) {
    targetDeg = newTargetDeg;
    // Re-arm from TIMEOUT state when a new target is received
    if (state == AxisState::TIMEOUT) {
      state = AxisState::IDLE;
    }
  }
};

static AxisController axes[4] = {
  {PIN_POT_AXIS1, &kfPot1},
  {PIN_POT_AXIS2, &kfPot2},
  {PIN_POT_AXIS3, &kfPot3},
  {PIN_POT_AXIS4, &kfPot4},
};

// ---------------------------------------------------------------------------
// ESP-NOW receive callback (runs in ISR context — keep it short)
// ---------------------------------------------------------------------------
static void onDataRecv(const esp_now_recv_info_t *recvInfo,
                       const uint8_t *data,
                       int dataLen) {
  if (dataLen != sizeof(RoboArmPacket)) return;

  portENTER_CRITICAL_ISR(&packetMux);
  memcpy((void *)&latestPacket, data, sizeof(RoboArmPacket));
  newDataAvailable = true;
  portEXIT_CRITICAL_ISR(&packetMux);
}

// ---------------------------------------------------------------------------
// setup()
// ---------------------------------------------------------------------------
void setup() {
  Serial.begin(115200);
  Serial.println("[Receiver] Booting...");

  // Motor output pins
  for (uint8_t i = 0; i < 4; i++) {
    pinMode(motors[i].fwd, OUTPUT);
    pinMode(motors[i].rev, OUTPUT);
    motorStop(i);
  }

  // Potentiometer input pins (input-only GPIOs — no pinMode needed, but explicit is fine)
  pinMode(PIN_POT_AXIS1, INPUT);
  pinMode(PIN_POT_AXIS2, INPUT);
  pinMode(PIN_POT_AXIS3, INPUT);
  pinMode(PIN_POT_AXIS4, INPUT);

  // Wi-Fi in STA mode (required for ESP-NOW)
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();

  // ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("[Receiver] ESP-NOW init FAILED — halting.");
    while (true) { delay(1000); }
  }
  esp_now_register_recv_cb(onDataRecv);

  Serial.println("[Receiver] Ready. MAC: " + WiFi.macAddress());
}

// ---------------------------------------------------------------------------
// loop()
// ---------------------------------------------------------------------------
void loop() {
  // Atomically consume new packet if available
  bool gotNew = false;
  RoboArmPacket pkt;

  portENTER_CRITICAL(&packetMux);
  if (newDataAvailable) {
    pkt             = latestPacket;
    newDataAvailable = false;
    gotNew          = true;
  }
  portEXIT_CRITICAL(&packetMux);

  if (gotNew) {
    axes[0].setTarget(pkt.axis1_angle);
    axes[1].setTarget(pkt.axis2_angle);
    axes[2].setTarget(pkt.axis3_angle);
    axes[3].setTarget(pkt.axis4_angle);
  }

  // Run state machine for each axis (non-blocking)
  for (uint8_t i = 0; i < 4; i++) {
    axes[i].update(i);
  }
}
