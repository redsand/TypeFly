#include <Wire.h>

// Motor pins (PWM)
const int PIN_FL = 3;
const int PIN_FR = 5;
const int PIN_RL = 6;
const int PIN_RR = 9;

// IMU (MPU6050) address
const int MPU_ADDR = 0x68;

// Safety
const unsigned long WATCHDOG_MS = 500;

// Tethered gains (deg -> PWM delta)
float PITCH_GAIN = 4.0;
float ROLL_GAIN = 4.0;

// PID gains for UNTETHERED
float KP_PITCH = 2.0;
float KI_PITCH = 0.1;
float KD_PITCH = 0.05;
float KP_ROLL = 2.0;
float KI_ROLL = 0.1;
float KD_ROLL = 0.05;

struct PIDState {
  float kp;
  float ki;
  float kd;
  float integral;
  float prev_error;
};

PIDState pid_pitch = {KP_PITCH, KI_PITCH, KD_PITCH, 0.0, 0.0};
PIDState pid_roll = {KP_ROLL, KI_ROLL, KD_ROLL, 0.0, 0.0};

bool armed = false;
bool stop_requested = false;
String mode = "TETHERED";

int throttle_cmd = 0;
float pitch_cmd = 0.0;
float roll_cmd = 0.0;

unsigned long last_cmd_ms = 0;
unsigned long last_telemetry_ms = 0;

float pitch_deg = 0.0;
float roll_deg = 0.0;

void writeMotors(int fl, int fr, int rl, int rr) {
  fl = constrain(fl, 0, 255);
  fr = constrain(fr, 0, 255);
  rl = constrain(rl, 0, 255);
  rr = constrain(rr, 0, 255);

  analogWrite(PIN_FL, fl);
  analogWrite(PIN_FR, fr);
  analogWrite(PIN_RL, rl);
  analogWrite(PIN_RR, rr);
}

void stopMotors() {
  writeMotors(0, 0, 0, 0);
}

float readPID(PIDState &pid, float error, float dt) {
  pid.integral += error * dt;
  float derivative = (error - pid.prev_error) / dt;
  pid.prev_error = error;
  return pid.kp * error + pid.ki * pid.integral + pid.kd * derivative;
}

void imuInit() {
  Wire.begin();
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission(true);
}

void imuRead(float &pitch_out, float &roll_out) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 14, true);

  int16_t accel_x = Wire.read() << 8 | Wire.read();
  int16_t accel_y = Wire.read() << 8 | Wire.read();
  int16_t accel_z = Wire.read() << 8 | Wire.read();
  Wire.read();
  Wire.read();
  int16_t gyro_x = Wire.read() << 8 | Wire.read();
  int16_t gyro_y = Wire.read() << 8 | Wire.read();
  int16_t gyro_z = Wire.read() << 8 | Wire.read();

  float ax = accel_x / 16384.0;
  float ay = accel_y / 16384.0;
  float az = accel_z / 16384.0;

  float pitch_acc = atan2(ay, az) * 180.0 / PI;
  float roll_acc = atan2(-ax, sqrt(ay * ay + az * az)) * 180.0 / PI;

  static float pitch_est = 0.0;
  static float roll_est = 0.0;
  static unsigned long last_ms = 0;
  unsigned long now_ms = millis();
  float dt = (now_ms - last_ms) / 1000.0;
  if (last_ms == 0 || dt <= 0.0) {
    pitch_est = pitch_acc;
    roll_est = roll_acc;
    last_ms = now_ms;
  }

  float gx = gyro_x / 131.0;
  float gy = gyro_y / 131.0;

  const float alpha = 0.98;
  pitch_est = alpha * (pitch_est + gx * dt) + (1 - alpha) * pitch_acc;
  roll_est = alpha * (roll_est + gy * dt) + (1 - alpha) * roll_acc;

  pitch_out = pitch_est;
  roll_out = roll_est;
}

void processCommand(String line) {
  line.trim();
  if (line.length() == 0) {
    return;
  }

  if (line.startsWith("ARM")) {
    int value = line.substring(3).toInt();
    armed = (value == 1);
    if (!armed) {
      stopMotors();
    }
    return;
  }

  if (line.startsWith("STOP")) {
    stop_requested = true;
    stopMotors();
    return;
  }

  if (line.startsWith("CMD")) {
    stop_requested = false;
    int first = line.indexOf(' ');
    int second = line.indexOf(' ', first + 1);
    int third = line.indexOf(' ', second + 1);
    int fourth = line.indexOf(' ', third + 1);
    if (first < 0 || second < 0 || third < 0 || fourth < 0) {
      return;
    }

    mode = line.substring(first + 1, second);
    throttle_cmd = line.substring(second + 1, third).toInt();
    pitch_cmd = line.substring(third + 1, fourth).toFloat();
    roll_cmd = line.substring(fourth + 1).toFloat();
    last_cmd_ms = millis();
  }
}

void applyMixer(int throttle_pwm, float pitch_delta, float roll_delta) {
  int fl = throttle_pwm + pitch_delta + roll_delta;
  int fr = throttle_pwm + pitch_delta - roll_delta;
  int rl = throttle_pwm - pitch_delta + roll_delta;
  int rr = throttle_pwm - pitch_delta - roll_delta;
  writeMotors(fl, fr, rl, rr);
}

void setup() {
  Serial.begin(115200);
  pinMode(PIN_FL, OUTPUT);
  pinMode(PIN_FR, OUTPUT);
  pinMode(PIN_RL, OUTPUT);
  pinMode(PIN_RR, OUTPUT);
  stopMotors();
  imuInit();
  last_cmd_ms = millis();
}

void loop() {
  static String buffer = "";
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n') {
      processCommand(buffer);
      buffer = "";
    } else if (c != '\r') {
      buffer += c;
    }
  }

  unsigned long now_ms = millis();
  if (now_ms - last_cmd_ms > WATCHDOG_MS) {
    stopMotors();
  }

  imuRead(pitch_deg, roll_deg);

  if (!armed || stop_requested) {
    stopMotors();
  } else if (mode == "TETHERED") {
    float p_delta = pitch_cmd * PITCH_GAIN;
    float r_delta = roll_cmd * ROLL_GAIN;
    applyMixer(throttle_cmd, p_delta, r_delta);
  } else {
    static unsigned long last_pid_ms = 0;
    float dt = 0.01;
    if (last_pid_ms != 0) {
      dt = (now_ms - last_pid_ms) / 1000.0;
      if (dt <= 0.0) {
        dt = 0.01;
      }
    }
    last_pid_ms = now_ms;
    float p_error = pitch_cmd - pitch_deg;
    float r_error = roll_cmd - roll_deg;
    float p_delta = readPID(pid_pitch, p_error, dt);
    float r_delta = readPID(pid_roll, r_error, dt);
    applyMixer(throttle_cmd, p_delta, r_delta);
  }

  if (now_ms - last_telemetry_ms > 100) {
    Serial.print("ATT ");
    Serial.print(pitch_deg, 2);
    Serial.print(" ");
    Serial.print(roll_deg, 2);
    Serial.print(" ");
    Serial.print(armed ? 1 : 0);
    Serial.print(" ");
    Serial.println(mode);
    last_telemetry_ms = now_ms;
  }
}
