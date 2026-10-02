#include <Wire.h>
#include <string.h>

const uint32_t LOOP_HZ = 100;  // matches BNO055 fusion output rate
const uint32_t LOOP_PERIOD_US = 1000000UL / LOOP_HZ;

// BNO055
#define BNO055_ADDR 0x28  // 0x29 if ADR pin is high
#define I2C_CLOCK_HZ 400000
#define BNO_FUSION_MODE BNO_MODE_IMU  // BNO_MODE_NDOF adds magnetometer heading
const bool BNO_USE_EXT_CRYSTAL = false;

// Sensor -> body axis: index of the BNO axis feeding body X/Y/Z, and its sign
const int AXIS_MAP[3] = {1, 0, 2};
const float AXIS_SIGN[3] = {1.0, -1.0, -1.0};

// Motors
const uint32_t PWM_FREQ_HZ = 400;
const int PWM_BITS = 16;
const int M1_PIN = 2;  // Front-Right (CCW)
const int M2_PIN = 3;  // Rear-Right (CW)
const int M3_PIN = 4;  // Rear-Left (CCW)
const int M4_PIN = 5;  // Front-Left (CW)
const uint16_t ESC_MIN_US = 1000;
const uint16_t ESC_MAX_US = 2000;
const uint16_t ESC_IDLE_US = 1060;
const uint16_t ESC_CEIL_US = 1900;
const float YAW_MIX_SIGN = 1.0;

// Pilot limits
const float MAX_ANGLE_DEG = 20.0;
const float MAX_YAW_RATE_DPS = 120.0;
const float MAX_THROTTLE = 0.80;

// PID gains. Values pushed over serial are RAM only; copy good ones here.
float Kp_roll = 0.0, Ki_roll = 0.0, Kd_roll = 0.142;    // 0.2, 0.3, 0.05
float Kp_pitch = 0.0, Ki_pitch = 0.0, Kd_pitch = 0.185; // 0.2, 0.3, 0.05
float Kp_yaw = 0.0, Ki_yaw = 0.0, Kd_yaw = 0.0;         // 0.3, 0.05, 0.00015
const float GAIN_P_MAX = 5.0;
const float GAIN_D_MAX = 2.0;
const float GAIN_YAWD_MAX = 0.01;
const float I_LIMIT = 25.0;

// Failsafe
const uint32_t LINK_TIMEOUT_MS = 400;
const float FAILSAFE_DESCENT = 0.25;  // throttle fraction lost per second
const uint32_t FAILSAFE_KILL_MS = 6000;

// #define ESC_CALIBRATION_MODE  // MAX for 8 s then MIN, forever
// #define PRINT_DEBUG           // text output instead of telemetry packets

// BNO055 registers
#define BNO_CHIP_ID 0x00
#define BNO_PAGE_ID 0x07
#define BNO_GYR_DATA_X 0x14  // gyro XYZ, then Euler heading/roll/pitch
#define BNO_GRV_DATA_X 0x2E  // fused gravity XYZ
#define BNO_CALIB_STAT 0x35
#define BNO_UNIT_SEL 0x3B
#define BNO_OPR_MODE 0x3D
#define BNO_PWR_MODE 0x3E
#define BNO_SYS_TRIGGER 0x3F
#define BNO_MODE_CONFIG 0x00
#define BNO_MODE_IMU 0x08
#define BNO_MODE_NDOF 0x0C
const float GYR_LSB_PER_DPS = 16.0;
const float EUL_LSB_PER_DEG = 16.0;

// Serial protocol (little-endian, CRC-8 poly 0x07 over bytes 2..n-2):
//   CTRL     A5 5A | flags(arm,kill) | roll,pitch cdeg i16 | yawrate cdps i16 | thr 0..1000 u16 | seq u16 | crc  (14 B)
//   SETGAINS A5 5D | 9x f32 Kp/Ki/Kd roll,pitch,yaw | crc  (39 B)
//   TLM      5A A5 | state | roll,pitch,yaw cdeg i16 | m1..m4 us u16 | loop_hz u16 | err | seq u16 | 9x f32 gains | crc  (59 B)
const uint8_t UP_HDR0 = 0xA5;
const uint8_t UP_HDR1 = 0x5A;
const uint8_t UP2_HDR1 = 0x5D;
const uint8_t DN_HDR0 = 0x5A;
const uint8_t DN_HDR1 = 0xA5;
const int UP_LEN = 14;
const int UP2_LEN = 39;
const int DN_LEN = 59;

const uint8_t ERR_IMU = 0x01;
const uint8_t ERR_CRC = 0x02;
const uint8_t ERR_CAL = 0x04;

enum State { DISARMED = 0, ARMED = 1, FAILSAFE = 2 };
int state = DISARMED;

// IMU
float GyroX = 0, GyroY = 0, GyroZ = 0;  // deg/s, body frame
float roll_IMU = 0, pitch_IMU = 0, yaw_IMU = 0;
int sysCal = 0, gyroCal = 0, accCal = 0, magCal = 0;  // 0..3 from CALIB_STAT
bool imu_ok = false;

// Control
float roll_des = 0, pitch_des = 0, yawrate_des = 0, thro_des = 0;
float roll_PID = 0, pitch_PID = 0, yaw_PID = 0;
float integral_roll = 0, integral_pitch = 0, integral_yaw = 0;
float error_yaw_prev = 0;
uint16_t m1_us = ESC_MIN_US, m2_us = ESC_MIN_US, m3_us = ESC_MIN_US, m4_us = ESC_MIN_US;

// Timing
float dt = 1.0 / LOOP_HZ;
uint32_t loop_prev_us = 0;
uint32_t measured_loop_hz = 0;
uint32_t last_calib_check_ms = 0;
uint32_t last_telemetry_ms = 0;
uint32_t last_debug_ms = 0;

// Link
uint32_t last_packet_ms = 0;
uint32_t failsafe_start_ms = 0;
float failsafe_throttle = 0;
uint16_t rx_seq = 0;
uint8_t errflags = ERR_CAL;
uint8_t rxbuf[UP2_LEN];
int rxlen = 0;
int rxtarget = 0;

uint8_t crc8(uint8_t *data, int len) {
  uint8_t crc = 0x00;
  for (int i = 0; i < len; i++) {
    crc = crc ^ data[i];
    for (int bit = 0; bit < 8; bit++) {
      if (crc & 0x80) {
        crc = (crc << 1) ^ 0x07;
      } else {
        crc = crc << 1;
      }
    }
  }
  return crc;
}

// Little-endian helpers for packing/unpacking packets
int16_t readInt16(uint8_t *buf, int index) {
  return (int16_t)(buf[index] | (buf[index + 1] << 8));
}

uint16_t readUInt16(uint8_t *buf, int index) {
  return (uint16_t)(buf[index] | (buf[index + 1] << 8));
}

float readFloat(uint8_t *buf, int index) {
  float value;
  memcpy(&value, &buf[index], 4);
  return value;
}

void writeInt16(uint8_t *buf, int index, int16_t value) {
  buf[index] = value & 0xFF;
  buf[index + 1] = (value >> 8) & 0xFF;
}

void writeUInt16(uint8_t *buf, int index, uint16_t value) {
  buf[index] = value & 0xFF;
  buf[index + 1] = (value >> 8) & 0xFF;
}

void writeFloat(uint8_t *buf, int index, float value) {
  memcpy(&buf[index], &value, 4);
}

bool bnoWrite(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(BNO055_ADDR);
  Wire.write(reg);
  Wire.write(val);
  return Wire.endTransmission() == 0;
}

bool bnoRead(uint8_t reg, uint8_t *buf, uint8_t len) {
  Wire.beginTransmission(BNO055_ADDR);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) {
    return false;
  }
  if (Wire.requestFrom(BNO055_ADDR, (int)len) != len) {
    return false;
  }
  for (int i = 0; i < len; i++) {
    buf[i] = Wire.read();
  }
  return true;
}

bool bnoWaitForChip(int tries) {
  for (int i = 0; i < tries; i++) {
    uint8_t id = 0;
    if (bnoRead(BNO_CHIP_ID, &id, 1) && id == 0xA0) {
      return true;
    }
    delay(50);
  }
  return false;
}

bool bnoInit() {
  if (!bnoWaitForChip(10)) {
    return false;
  }
  bnoWrite(BNO_PAGE_ID, 0x00);
  bnoWrite(BNO_OPR_MODE, BNO_MODE_CONFIG);
  delay(25);
  bnoWrite(BNO_SYS_TRIGGER, 0x20);  // reset
  delay(700);
  if (!bnoWaitForChip(20)) {
    return false;
  }
  bnoWrite(BNO_PWR_MODE, 0x00);
  delay(15);
  bnoWrite(BNO_PAGE_ID, 0x00);
  bnoWrite(BNO_UNIT_SEL, 0x00);  // m/s^2, dps, degrees
  delay(10);
  if (BNO_USE_EXT_CRYSTAL) {
    bnoWrite(BNO_SYS_TRIGGER, 0x80);
  } else {
    bnoWrite(BNO_SYS_TRIGGER, 0x00);
  }
  delay(10);
  bnoWrite(BNO_OPR_MODE, BNO_FUSION_MODE);
  delay(30);
  return true;
}

bool getIMUdata() {
  uint8_t gyroBytes[12];  // gyro XYZ (6 bytes) + Euler heading/roll/pitch (6 bytes)
  uint8_t gravBytes[6];
  if (!bnoRead(BNO_GYR_DATA_X, gyroBytes, 12)) {
    return false;
  }
  if (!bnoRead(BNO_GRV_DATA_X, gravBytes, 6)) {
    return false;
  }

  float rawGyro[3];
  float rawGrav[3];
  for (int i = 0; i < 3; i++) {
    rawGyro[i] = readInt16(gyroBytes, i * 2) / GYR_LSB_PER_DPS;
    rawGrav[i] = readInt16(gravBytes, i * 2);
  }

  // Remap sensor axes to body axes
  float gyro[3];
  float grav[3];
  for (int i = 0; i < 3; i++) {
    gyro[i] = AXIS_SIGN[i] * rawGyro[AXIS_MAP[i]];
    grav[i] = AXIS_SIGN[i] * rawGrav[AXIS_MAP[i]];
  }
  GyroX = gyro[0];
  GyroY = gyro[1];
  GyroZ = gyro[2];

  // +roll = right wing down, +pitch = nose up
  float horizontal = sqrt(grav[1] * grav[1] + grav[2] * grav[2]);
  roll_IMU = atan2(grav[1], grav[2]) * RAD_TO_DEG;
  pitch_IMU = atan2(-grav[0], horizontal) * RAD_TO_DEG;

  float heading = readInt16(gyroBytes, 6) / EUL_LSB_PER_DEG;  // 0..360
  if (heading > 180.0) {
    heading = heading - 360.0;
  }
  yaw_IMU = heading;
  return true;
}

// Arming is blocked until the chip reports gyro calibration = 3
void updateCalibStatus() {
  if (millis() - last_calib_check_ms < 100) {
    return;
  }
  last_calib_check_ms = millis();

  uint8_t calib = 0;
  if (!bnoRead(BNO_CALIB_STAT, &calib, 1)) {
    return;
  }
  // CALIB_STAT holds four 2-bit fields: sys, gyro, accel, mag
  sysCal = (calib >> 6) & 0x03;
  gyroCal = (calib >> 4) & 0x03;
  accCal = (calib >> 2) & 0x03;
  magCal = calib & 0x03;

  if (gyroCal == 3) {
    errflags = errflags & ~ERR_CAL;
  } else {
    errflags = errflags | ERR_CAL;
  }
}

void resetPID() {
  integral_roll = 0;
  integral_pitch = 0;
  integral_yaw = 0;
  error_yaw_prev = 0;
}

void controlANGLE() {
  bool lowThrottle = thro_des < 0.03;

  // Roll
  float error_roll = roll_des - roll_IMU;
  if (lowThrottle) {
    integral_roll = 0;
  } else {
    integral_roll = constrain(integral_roll + error_roll * dt, -I_LIMIT, I_LIMIT);
  }
  roll_PID = 0.01 * (Kp_roll * error_roll + Ki_roll * integral_roll - Kd_roll * GyroX);

  // Pitch
  float error_pitch = pitch_des - pitch_IMU;
  if (lowThrottle) {
    integral_pitch = 0;
  } else {
    integral_pitch = constrain(integral_pitch + error_pitch * dt, -I_LIMIT, I_LIMIT);
  }
  pitch_PID = 0.01 * (Kp_pitch * error_pitch + Ki_pitch * integral_pitch - Kd_pitch * GyroY);

  // Yaw (rate)
  float error_yaw = yawrate_des - GyroZ;
  if (lowThrottle) {
    integral_yaw = 0;
  } else {
    integral_yaw = constrain(integral_yaw + error_yaw * dt, -I_LIMIT, I_LIMIT);
  }
  float derivative_yaw = (error_yaw - error_yaw_prev) / dt;
  yaw_PID = 0.01 * (Kp_yaw * error_yaw + Ki_yaw * integral_yaw + Kd_yaw * derivative_yaw);
  error_yaw_prev = error_yaw;
}

// Converts a 0..1 motor command to a pulse width between idle and ceiling
uint16_t commandToPulse(float command) {
  command = constrain(command, 0.0f, 1.0f);
  float us = ESC_MIN_US + command * (ESC_MAX_US - ESC_MIN_US);
  if (us < ESC_IDLE_US) {
    us = ESC_IDLE_US;
  }
  if (us > ESC_CEIL_US) {
    us = ESC_CEIL_US;
  }
  return (uint16_t)us;
}

// Quad X: +roll -> left motors up, +pitch -> front motors up, +yaw -> CCW motors up
void mixMotors() {
  float yaw = YAW_MIX_SIGN * yaw_PID;
  float m1 = thro_des + pitch_PID - roll_PID + yaw;  // Front-Right
  float m2 = thro_des - pitch_PID - roll_PID - yaw;  // Rear-Right
  float m3 = thro_des - pitch_PID + roll_PID + yaw;  // Rear-Left
  float m4 = thro_des + pitch_PID + roll_PID - yaw;  // Front-Left
  m1_us = commandToPulse(m1);
  m2_us = commandToPulse(m2);
  m3_us = commandToPulse(m3);
  m4_us = commandToPulse(m4);
}

// Converts a pulse width in microseconds to an analogWrite duty value
uint32_t pulseToDuty(uint16_t us) {
  float period_us = 1000000.0 / PWM_FREQ_HZ;
  float fullScale = (1UL << PWM_BITS) - 1;
  return (uint32_t)(us / period_us * fullScale);
}

void writeMotors() {
  analogWrite(M1_PIN, pulseToDuty(m1_us));
  analogWrite(M2_PIN, pulseToDuty(m2_us));
  analogWrite(M3_PIN, pulseToDuty(m3_us));
  analogWrite(M4_PIN, pulseToDuty(m4_us));
}

void setAllMotors(uint16_t us) {
  m1_us = us;
  m2_us = us;
  m3_us = us;
  m4_us = us;
}

void handleCtrlPacket(uint8_t *packet) {
  bool armRequest = (packet[2] & 0x01) != 0;
  bool killRequest = (packet[2] & 0x02) != 0;
  int16_t rollCmd = readInt16(packet, 3);
  int16_t pitchCmd = readInt16(packet, 5);
  int16_t yawCmd = readInt16(packet, 7);
  uint16_t throttleCmd = readUInt16(packet, 9);
  rx_seq = readUInt16(packet, 11);

  bool calibrated = (errflags & ERR_CAL) == 0;

  if (killRequest) {
    state = DISARMED;
    resetPID();
  } else if (state == DISARMED) {
    if (armRequest && throttleCmd == 0 && imu_ok && calibrated) {
      resetPID();
      state = ARMED;
    }
  } else {
    // Armed or in failsafe
    if (!armRequest) {
      state = DISARMED;
      resetPID();
    } else if (state == FAILSAFE) {
      state = ARMED;  // link recovered
    }
  }

  if (state == ARMED) {
    roll_des = constrain(rollCmd / 100.0f, -MAX_ANGLE_DEG, MAX_ANGLE_DEG);
    pitch_des = constrain(pitchCmd / 100.0f, -MAX_ANGLE_DEG, MAX_ANGLE_DEG);
    yawrate_des = constrain(yawCmd / 100.0f, -MAX_YAW_RATE_DPS, MAX_YAW_RATE_DPS);
    thro_des = constrain(throttleCmd / 1000.0f, 0.0f, 1.0f) * MAX_THROTTLE;
  }
  last_packet_ms = millis();
}

// Does not refresh last_packet_ms: only CTRL keeps the link alive
void handleGainsPacket(uint8_t *packet) {
  float g[9];
  for (int i = 0; i < 9; i++) {
    g[i] = readFloat(packet, 2 + i * 4);
    if (isnan(g[i]) || isinf(g[i]) || g[i] < 0.0) {
      return;
    }
  }
  Kp_roll = constrain(g[0], 0.0f, GAIN_P_MAX);
  Ki_roll = constrain(g[1], 0.0f, GAIN_P_MAX);
  Kd_roll = constrain(g[2], 0.0f, GAIN_D_MAX);
  Kp_pitch = constrain(g[3], 0.0f, GAIN_P_MAX);
  Ki_pitch = constrain(g[4], 0.0f, GAIN_P_MAX);
  Kd_pitch = constrain(g[5], 0.0f, GAIN_D_MAX);
  Kp_yaw = constrain(g[6], 0.0f, GAIN_P_MAX);
  Ki_yaw = constrain(g[7], 0.0f, GAIN_P_MAX);
  Kd_yaw = constrain(g[8], 0.0f, GAIN_YAWD_MAX);
}

void processPacket() {
  uint8_t expectedCrc = crc8(&rxbuf[2], rxtarget - 3);
  uint8_t receivedCrc = rxbuf[rxtarget - 1];
  if (expectedCrc != receivedCrc) {
    errflags = errflags | ERR_CRC;
    return;
  }
  errflags = errflags & ~ERR_CRC;
  if (rxtarget == UP_LEN) {
    handleCtrlPacket(rxbuf);
  } else {
    handleGainsPacket(rxbuf);
  }
}

void readSerial() {
  while (Serial.available() > 0) {
    uint8_t b = Serial.read();

    if (rxlen == 0) {
      // Waiting for the first header byte
      if (b == UP_HDR0) {
        rxbuf[0] = b;
        rxlen = 1;
      }
    } else if (rxlen == 1) {
      // Second header byte tells us which packet this is
      if (b == UP_HDR1) {
        rxtarget = UP_LEN;
        rxbuf[1] = b;
        rxlen = 2;
      } else if (b == UP2_HDR1) {
        rxtarget = UP2_LEN;
        rxbuf[1] = b;
        rxlen = 2;
      } else {
        rxlen = 0;
      }
    } else {
      rxbuf[rxlen] = b;
      rxlen++;
      if (rxlen == rxtarget) {
        processPacket();
        rxlen = 0;
      }
    }
  }
}

void sendTelemetry() {
  if (millis() - last_telemetry_ms < 50) {  // 20 Hz
    return;
  }
  last_telemetry_ms = millis();

  uint8_t t[DN_LEN];
  t[0] = DN_HDR0;
  t[1] = DN_HDR1;
  t[2] = state;
  writeInt16(t, 3, (int16_t)(roll_IMU * 100.0));
  writeInt16(t, 5, (int16_t)(pitch_IMU * 100.0));
  writeInt16(t, 7, (int16_t)(yaw_IMU * 100.0));
  writeUInt16(t, 9, m1_us);
  writeUInt16(t, 11, m2_us);
  writeUInt16(t, 13, m3_us);
  writeUInt16(t, 15, m4_us);
  writeUInt16(t, 17, measured_loop_hz);
  t[19] = errflags;
  writeUInt16(t, 20, rx_seq);

  float gains[9] = {Kp_roll, Ki_roll, Kd_roll, Kp_pitch, Ki_pitch, Kd_pitch, Kp_yaw, Ki_yaw, Kd_yaw};
  for (int i = 0; i < 9; i++) {
    writeFloat(t, 22 + i * 4, gains[i]);
  }

  t[58] = crc8(&t[2], DN_LEN - 3);
  Serial.write(t, DN_LEN);
}

// On link loss: level out and ramp throttle down, hard disarm on timeout
void checkFailsafe() {
  uint32_t now = millis();
  bool linkLost = (now - last_packet_ms) > LINK_TIMEOUT_MS;

  if (state == ARMED && linkLost) {
    state = FAILSAFE;
    failsafe_start_ms = now;
    failsafe_throttle = thro_des;
  }

  if (state == FAILSAFE) {
    roll_des = 0;
    pitch_des = 0;
    yawrate_des = 0;
    failsafe_throttle = failsafe_throttle - FAILSAFE_DESCENT * MAX_THROTTLE * dt;
    thro_des = constrain(failsafe_throttle, 0.0f, MAX_THROTTLE);

    bool timedOut = (now - failsafe_start_ms) > FAILSAFE_KILL_MS;
    if (thro_des <= 0.0 || timedOut) {
      state = DISARMED;
      resetPID();
    }
  }
}

void printDebug() {
  if (millis() - last_debug_ms <= 100) {
    return;
  }
  last_debug_ms = millis();

  Serial.print("st=");
  Serial.print(state);
  Serial.print(" hz=");
  Serial.print(measured_loop_hz);
  Serial.print(" cal(s/g/a/m)=");
  Serial.print(sysCal);
  Serial.print("/");
  Serial.print(gyroCal);
  Serial.print("/");
  Serial.print(accCal);
  Serial.print("/");
  Serial.print(magCal);
  Serial.print(" r=");
  Serial.print(roll_IMU, 2);
  Serial.print(" p=");
  Serial.print(pitch_IMU, 2);
  Serial.print(" y=");
  Serial.print(yaw_IMU, 2);
  Serial.print(" gx=");
  Serial.print(GyroX, 2);
  Serial.print(" gy=");
  Serial.print(GyroY, 2);
  Serial.print(" gz=");
  Serial.print(GyroZ, 2);
  Serial.print(" m=");
  Serial.print(m1_us);
  Serial.print(" ");
  Serial.print(m2_us);
  Serial.print(" ");
  Serial.print(m3_us);
  Serial.print(" ");
  Serial.println(m4_us);
}

// LED: slow blink = disarmed, solid = armed, fast blink = failsafe
void updateLED() {
  if (state == ARMED) {
    digitalWrite(LED_BUILTIN, HIGH);
    return;
  }

  int blinkPeriod = 500;
  if (state == FAILSAFE) {
    blinkPeriod = 80;
  }
  if ((millis() / blinkPeriod) % 2 == 1) {
    digitalWrite(LED_BUILTIN, HIGH);
  } else {
    digitalWrite(LED_BUILTIN, LOW);
  }
}

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);
  Serial.begin(500000);

  analogWriteResolution(PWM_BITS);
  pinMode(M1_PIN, OUTPUT);
  pinMode(M2_PIN, OUTPUT);
  pinMode(M3_PIN, OUTPUT);
  pinMode(M4_PIN, OUTPUT);
  analogWriteFrequency(M1_PIN, PWM_FREQ_HZ);
  analogWriteFrequency(M2_PIN, PWM_FREQ_HZ);
  analogWriteFrequency(M3_PIN, PWM_FREQ_HZ);
  analogWriteFrequency(M4_PIN, PWM_FREQ_HZ);
  setAllMotors(ESC_MIN_US);
  writeMotors();
  delay(500);  // let ESCs see a valid low signal first

#ifdef ESC_CALIBRATION_MODE
  while (true) {
    setAllMotors(ESC_MAX_US);
    writeMotors();
    digitalWrite(LED_BUILTIN, HIGH);
    delay(8000);
    setAllMotors(ESC_MIN_US);
    writeMotors();
    digitalWrite(LED_BUILTIN, LOW);
    delay(8000);
  }
#endif

  Wire.begin();
  Wire.setClock(I2C_CLOCK_HZ);
  delay(100);
  imu_ok = bnoInit();
  if (!imu_ok) {
    errflags = errflags | ERR_IMU;
  }

  digitalWrite(LED_BUILTIN, LOW);
  last_packet_ms = millis();
  loop_prev_us = micros();
}

void loop() {
  // Run at a fixed rate
  uint32_t now_us = micros();
  uint32_t elapsed = now_us - loop_prev_us;
  if (elapsed < LOOP_PERIOD_US) {
    return;
  }
  loop_prev_us = now_us;
  dt = elapsed / 1000000.0;
  if (dt > 0.05) {
    dt = 0.05;
  }
  measured_loop_hz = (uint32_t)(1.0 / dt + 0.5);

  if (getIMUdata()) {
    errflags = errflags & ~ERR_IMU;
    imu_ok = true;
    updateCalibStatus();
  } else {
    errflags = errflags | ERR_IMU;
    imu_ok = false;
    state = DISARMED;  // no attitude -> no flight
  }

  readSerial();
  checkFailsafe();

  if (state == ARMED || state == FAILSAFE) {
    controlANGLE();
    mixMotors();
  } else {
    roll_des = 0;
    pitch_des = 0;
    yawrate_des = 0;
    thro_des = 0;
    roll_PID = 0;
    pitch_PID = 0;
    yaw_PID = 0;
    resetPID();
    setAllMotors(ESC_MIN_US);
  }
  writeMotors();

#ifdef PRINT_DEBUG
  printDebug();
#else
  sendTelemetry();
#endif

  updateLED();
}
