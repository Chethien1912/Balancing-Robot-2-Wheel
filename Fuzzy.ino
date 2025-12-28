#include <Wire.h>
#include "Kalman.h"
#include <avr/io.h>
#include <avr/interrupt.h>

Kalman kalmanX;

// =================== PIN CONFIG ===================
const int LEFT_PWM  = 46;  // Mega: Timer5
const int LEFT_IN1  = 50;
const int LEFT_IN2  = 48;

const int RIGHT_PWM = 44;  // Mega: Timer5
const int RIGHT_IN1 = 40;
const int RIGHT_IN2 = 42;

// =================== ENCODERS ===================
volatile long leftencoder  = 0;
volatile long righencoder  = 0;

const int leftencoder_a = 3;
const int leftencoder_b = 5;
const int righencoder_a = 2;
const int righencoder_b = 4;

// =================== IMU ===================
float AcX, AcZ;
float Gxro, Gxrate;     // gyro deg/s
float pitch = 0.0f;     // deg
float pitch_offset_deg = 0.5f;  // chỉnh bias thủ công

uint32_t t_prev_us = 0;
uint8_t i2cData[14];

// =================== STATES (đang tính nhưng chưa dùng vào điều khiển) ===================
float theta, phi;
float thetadot, phidot;
float thetaold = 0, phiold = 0;

// =================== MOTOR CONFIG ===================
const int PWM_MAX   = 255;
const int PWM_MIN   = 20;   // min PWM để thắng ma sát (tune)
const int PWM_DEADBAND = 2; // |U| < deadband => dừng (tune)

// =================== FUZZY SCALE (CHUẨN HÓA VỀ [-1..1]) ===================
// e = góc nghiêng (rad). de = tốc độ góc (rad/s)
// E_MAX ~ 20deg = 0.349rad (tune theo xe)
// DE_MAX ~ 200deg/s = 3.49rad/s (tune theo gyro)
const float E_MAX  = 0.25f;  // rad
const float DE_MAX = 2.0f;   // rad/s

// U scale
const float U_MAX = 220.0f;  // scale ra PWM (tune 180..255)

// =================== I2C ===================
static inline void i2cWriteByte(uint8_t addr, uint8_t reg, uint8_t data) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  Wire.write(data);
  Wire.endTransmission(true);
}

static inline int i2cRead(uint8_t addr, uint8_t reg, uint8_t* data, int len) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return 1;
  Wire.requestFrom(addr, (uint8_t)len);
  for (int i = 0; i < len; i++) {
    if (!Wire.available()) return 2;
    data[i] = Wire.read();
  }
  return 0;
}

// =================== FUZZY UTILS ===================
static inline float tri(float x, float a, float b, float c) {
  if (x <= a || x >= c) return 0.0f;
  if (x == b) return 1.0f;
  if (x < b)  return (x - a) / (b - a);
  return (c - x) / (c - b);
}
static inline float trapL(float x, float a, float b) { // left-shoulder
  if (x <= a) return 1.0f;
  if (x >= b) return 0.0f;
  return (b - x) / (b - a);
}
static inline float trapR(float x, float a, float b) { // right-shoulder
  if (x <= a) return 0.0f;
  if (x >= b) return 1.0f;
  return (x - a) / (b - a);
}

// ===== 7 MF for [-1..1] =====
// NB, NM, NS, ZO, PS, PM, PB
static inline void fuzzify7(float x, float mu[7]) {
  // NB, PB dùng shoulder để không bị “mất lực” ở biên
  mu[0] = trapL(x, -1.0f, -0.6667f);
  mu[1] = tri  (x, -1.0f, -0.6667f, -0.3333f);
  mu[2] = tri  (x, -0.6667f, -0.3333f,  0.0f);
  mu[3] = tri  (x, -0.3333f,  0.0f,  0.3333f);
  mu[4] = tri  (x,  0.0f,  0.3333f,  0.6667f);
  mu[5] = tri  (x,  0.3333f,  0.6667f,  1.0f);
  mu[6] = trapR(x,  0.6667f,  1.0f);
}

// output MF (same set) for idx 0..6 at x in [-1..1]
static inline float mu_out7(int idx, float x) {
  switch (idx) {
    case 0: return trapL(x, -1.0f, -0.6667f);
    case 1: return tri  (x, -1.0f, -0.6667f, -0.3333f);
    case 2: return tri  (x, -0.6667f, -0.3333f,  0.0f);
    case 3: return tri  (x, -0.3333f,  0.0f,  0.3333f);
    case 4: return tri  (x,  0.0f,  0.3333f,  0.6667f);
    case 5: return tri  (x,  0.3333f,  0.6667f,  1.0f);
    case 6: return trapR(x,  0.6667f,  1.0f);
    default: return 0.0f;
  }
}

// ===== RULE TABLE (1..7) =====
const uint8_t rule_u[7][7] = {
  {7,7,6,6,6,5,4},
  {7,6,6,6,5,4,3},
  {6,6,5,4,3,2,2},
  {6,5,4,3,2,2,1},
  {6,4,3,2,2,1,1},
  {5,4,3,2,2,2,1},
  {4,3,2,2,2,1,1}
};

// Fuzzy-PD: input đã chuẩn hóa [-1..1]
static inline float fuzzyPD_norm(float eN, float deN) {
  float muE[7], muDE[7];
  fuzzify7(eN, muE);
  fuzzify7(deN, muDE);

  float num = 0.0f, den = 0.0f;

  // centroid by sampling 201 points: -1..1 step 0.01
  for (int i = 0; i <= 200; i++) {
    float x = -1.0f + 0.01f * i;
    float muAgg = 0.0f;

    for (int ie = 0; ie < 7; ie++) {
      float muEi = muE[ie];
      if (muEi <= 0) continue;

      for (int ide = 0; ide < 7; ide++) {
        float w = (muEi < muDE[ide]) ? muEi : muDE[ide]; // min
        if (w <= 0) continue;

        int outIdx = rule_u[ie][ide] - 1;               // 0..6
        float mout = mu_out7(outIdx, x);
        float clipped = (w < mout) ? w : mout;          // min(w, mout)
        if (clipped > muAgg) muAgg = clipped;           // max
      }
    }

    num += x * muAgg;
    den += muAgg;
  }

  if (den <= 1e-6f) return 0.0f;
  return num / den; // -1..1
}

// =================== MOTOR ===================
static inline void setMotor(int pwmPin, int in1, int in2, float u) {
  float au = fabs(u);

  // deadband -> stop
  if (au < PWM_DEADBAND) {
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
    analogWrite(pwmPin, 0);
    return;
  }

  int pwm = (int)au;
  if (pwm > 0) pwm = pwm + PWM_MIN;   // thêm minPWM khi thật sự chạy
  if (pwm > PWM_MAX) pwm = PWM_MAX;

  if (u > 0) { // forward
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
  } else {     // reverse
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
  }

  analogWrite(pwmPin, pwm);
}

static inline void motorcontrol(float L, float R) {
  setMotor(LEFT_PWM,  LEFT_IN1,  LEFT_IN2,  L);
  setMotor(RIGHT_PWM, RIGHT_IN1, RIGHT_IN2, R);
}

// =================== ISR ===================
void left_isr() {
  // Nếu chiều encoder ngược thực tế -> đảo ++/-- tại đây
  if (digitalRead(leftencoder_b)) leftencoder--;
  else leftencoder++;
}

void righ_isr() {
  // Nếu chiều encoder ngược thực tế -> đảo ++/-- tại đây
  if (digitalRead(righencoder_b)) righencoder++;
  else righencoder--;
}

// =================== READ MPU ===================
static inline void READ_MPU(float &dt) {
  // Read 14 bytes from ACCEL_XOUT_H
  i2cRead(0x68, 0x3B, i2cData, 14);

  int16_t ax = (int16_t)((i2cData[0] << 8) | i2cData[1]);
  int16_t az = (int16_t)((i2cData[4] << 8) | i2cData[5]);
  int16_t gx = (int16_t)((i2cData[10] << 8) | i2cData[11]);

  AcX = (float)ax;
  AcZ = (float)az;

  // gyro: deg/s (FS=±250dps => 131 LSB/deg/s)
  Gxro   = -(float)gx;          // giữ dấu giống code bạn
  Gxrate = Gxro / 131.0f;

  // dt by micros
  uint32_t t_now = micros();
  dt = (t_now - t_prev_us) * 1e-6f;
  if (dt <= 0) dt = 0.005f;
  t_prev_us = t_now;

  float accAngle = atan2(AcX, AcZ) * RAD_TO_DEG;
  pitch = kalmanX.getAngle(accAngle, Gxrate, dt);
  pitch -= pitch_offset_deg;
}

// =================== PWM FREQ (OPTIONAL) ===================
// Pins 44/46 on Mega use Timer5. Set ~31kHz for smoother motor (optional)
static inline void setupTimer5_31kHz() {
  // Fast PWM 8-bit, prescaler = 1  => ~31.37kHz on Timer5
  // Arduino core sets Timer5 for analogWrite; we overwrite to high freq.
  TCCR5A = 0;
  TCCR5B = 0;
  // Fast PWM 8-bit: WGM50=1
  TCCR5A |= (1 << WGM50);
  // Non-inverting on OC5A/OC5B: COM5A1, COM5B1
  TCCR5A |= (1 << COM5A1) | (1 << COM5B1);
  // prescaler = 1
  TCCR5B |= (1 << CS50);
  // OC5A=46, OC5B=45 (nhưng bạn dùng 44/46; 44 là OC5C -> cũng set)
  TCCR5A |= (1 << COM5C1); // enable OC5C
}

// =================== SETUP ===================
void setup() {
  pinMode(LEFT_PWM,  OUTPUT);
  pinMode(LEFT_IN1,  OUTPUT);
  pinMode(LEFT_IN2,  OUTPUT);
  pinMode(RIGHT_PWM, OUTPUT);
  pinMode(RIGHT_IN1, OUTPUT);
  pinMode(RIGHT_IN2, OUTPUT);

  pinMode(leftencoder_a, INPUT_PULLUP);
  pinMode(leftencoder_b, INPUT_PULLUP);
  pinMode(righencoder_a, INPUT_PULLUP);
  pinMode(righencoder_b, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(leftencoder_a), left_isr, RISING);
  attachInterrupt(digitalPinToInterrupt(righencoder_a), righ_isr, RISING);

  Wire.begin();
  Wire.setClock(400000);

  // Wake MPU6050
  i2cWriteByte(0x68, 0x6B, 0x00); // PWR_MGMT_1 = 0
  delay(50);

  // (optional) set gyro full scale = ±250dps (0x1B=0)
  i2cWriteByte(0x68, 0x1B, 0x00);
  // (optional) set accel full scale = ±2g (0x1C=0)
  i2cWriteByte(0x68, 0x1C, 0x00);

  // Read initial accel to set Kalman angle
  i2cRead(0x68, 0x3B, i2cData, 6);
  int16_t ax0 = (int16_t)((i2cData[0] << 8) | i2cData[1]);
  int16_t az0 = (int16_t)((i2cData[4] << 8) | i2cData[5]);
  AcX = (float)ax0;
  AcZ = (float)az0;

  float accAngle0 = atan2(AcX, AcZ) * RAD_TO_DEG;
  kalmanX.setAngle(accAngle0);

  // Init time
  t_prev_us = micros();

  // Optional: high-frequency PWM (nếu driver TB6612 thích 20k-30k)
  setupTimer5_31kHz();

  // Stop motors
  motorcontrol(0, 0);
}

// =================== LOOP ===================
void loop() {
  float dt;
  READ_MPU(dt);

  // Safety
  if (fabs(pitch) > 45.0f) {
    motorcontrol(0, 0);
    return;
  }

  // ===== Encoder states (hiện tại chưa dùng vào điều khiển) =====
  // dt dùng đúng theo READ_MPU
  theta = 0.55f * (leftencoder + righencoder);
  phi   = 0.25f * (leftencoder - righencoder);

  thetadot = (theta - thetaold) / dt;
  phidot   = (phi - phiold) / dt;

  thetaold = theta;
  phiold   = phi;

  // ===== Fuzzy PD =====
  // e: pitch (rad), de: gyro rate (rad/s)
  float e  = -pitch  * DEG_TO_RAD;      // rad
  float de = -Gxrate * DEG_TO_RAD;      // rad/s

  // normalize to [-1..1]
  float eN  = e  / E_MAX;
  float deN = de / DE_MAX;
  if (eN  > 1) eN  = 1; if (eN  < -1) eN  = -1;
  if (deN > 1) deN = 1; if (deN < -1) deN = -1;

  float uN = fuzzyPD_norm(eN, deN);     // -1..1
  float U  = -uN * U_MAX;               // PWM-like command

  // 2 motor same command for balancing
  motorcontrol(U, U);
}
