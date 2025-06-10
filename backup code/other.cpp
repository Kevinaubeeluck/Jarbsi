#include <Arduino.h>
#include <SPI.h>
#include <TimerInterrupt_Generic.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <step.h>
#include <WiFi.h>
#include <cmath>

// WiFi credentials
const char* ssid = "meow";
const char* password = "hello12345@";

// Constants
constexpr float G = 9.80665;
constexpr float DEG_TO_RAD = 0.0174533;
constexpr float MAX_ACCEL = 75.0;
constexpr float MAX_SPEED = 10.0;
constexpr float MAX_ANGLE_ERROR = 0.03;
constexpr float MIN_TILT_OFFSET = 0.13;

// PID Constants
float Kp = 3000, Ki = 0, Kd = 0.01;
float kmp = 5, kmi = 0, kmd = 0;

// State variables
float tilt_x = 0, tilt_x_prev = 0;
float actual_motor_speed = 0, actual_motor_speed_prev = 0;
float motorspeed = 0, accel = 0;
float angle_found = 0;
float dt = 0, last_time = 0, now = 0;

// PID tracking
float integral = 0, previous = 0;
float integral2 = 0, previous2 = 0;

// Pins
const int STEPPER1_DIR_PIN = 16;
const int STEPPER1_STEP_PIN = 17;
const int STEPPER2_DIR_PIN = 4;
const int STEPPER2_STEP_PIN = 14;
const int STEPPER_EN_PIN = 15;
const int ADC_CS_PIN = 5;
const int ADC_SCK_PIN = 18;
const int ADC_MISO_PIN = 19;
const int ADC_MOSI_PIN = 23;
const int TOGGLE_PIN = 32;
const int LED = 2;

const int PRINT_INTERVAL = 500;
const int LOOP_INTERVAL = 10;
const int STEPPER_INTERVAL_US = 20;

ESP32Timer ITimer(3);
Adafruit_MPU6050 mpu;
step step1(STEPPER_INTERVAL_US, STEPPER1_STEP_PIN, STEPPER1_DIR_PIN);
step step2(STEPPER_INTERVAL_US, STEPPER2_STEP_PIN, STEPPER2_DIR_PIN);

bool TimerHandler(void* timerNo) {
  static bool toggle = false;
  step1.runStepper();
  step2.runStepper();
  digitalWrite(TOGGLE_PIN, toggle);
  toggle = !toggle;
  return true;
}

float pid_1(float error) {
  if (dt <= 0) return 0;
  float integral_max = 5;
  integral += error * dt;
  integral = constrain(integral, -integral_max, integral_max);
  float derivative = (error - previous) / dt;
  previous = error;
  float output = (Kp * error) + (Ki * integral) + (Kd * derivative);
  return constrain(output, -MAX_ACCEL, MAX_ACCEL);
}

float pid_2(float error) {
  if (dt <= 0) return 0;
  float integral_max = 2;
  integral2 += error * dt;
  integral2 = constrain(integral2, -integral_max, integral_max);
  float derivative2 = (error - previous2) / dt;
  previous2 = error;
  float output = (kmp * error) + (kmi * integral2) + (kmd * derivative2);
  return constrain(output, -MAX_ANGLE_ERROR, MAX_ANGLE_ERROR);
}

void setup() {
  Serial.begin(115200);
  pinMode(LED, OUTPUT);
  pinMode(TOGGLE_PIN, OUTPUT);
  pinMode(STEPPER_EN_PIN, OUTPUT);
  digitalWrite(STEPPER_EN_PIN, false);

  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) delay(10);
  }

  mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
  mpu.setGyroRange(MPU6050_RANGE_250_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_44_HZ);

  if (!ITimer.attachInterruptInterval(STEPPER_INTERVAL_US, TimerHandler)) {
    Serial.println("Failed to start stepper interrupt");
    while (1) delay(10);
  }

  SPI.begin(ADC_SCK_PIN, ADC_MISO_PIN, ADC_MOSI_PIN, ADC_CS_PIN);
}

void loop() {
  static unsigned long printTimer = 0;
  static unsigned long loopTimer = 0;

  if (millis() > loopTimer) {
    loopTimer += LOOP_INTERVAL;
    now = millis();
    dt = (now - last_time) / 1000.0;
    last_time = now;

    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    actual_motor_speed_prev = actual_motor_speed;
    actual_motor_speed = step1.getSpeedRad();
    float error_speed = -2.0 - (actual_motor_speed / 10.0);
    float angle_adjust = pid_2(error_speed);

    if (actual_motor_speed < 5.0) angle_adjust = 0;
    angle_found = angle_adjust + MIN_TILT_OFFSET;

    float accel_angle = atan2(a.acceleration.x, a.acceleration.z);
    float gyro_rate_rad = g.gyro.y * DEG_TO_RAD;
    tilt_x = 0.98 * (tilt_x_prev + gyro_rate_rad * dt) + 0.02 * accel_angle;
    tilt_x_prev = tilt_x;

    float error = angle_found - tilt_x;
    accel = pid_1(error);

    motorspeed += accel * dt;
    motorspeed = constrain(motorspeed, -MAX_SPEED, MAX_SPEED);

    step1.setTargetSpeedRad(motorspeed);
    step2.setTargetSpeedRad(-motorspeed);
    step1.setAccelerationRad(accel);
    step2.setAccelerationRad(accel);
  }

  if (millis() > printTimer) {
    printTimer += PRINT_INTERVAL;
    Serial.print("MotorSpeed: ");
    Serial.print(actual_motor_speed);
    Serial.print(" | Target Angle: ");
    Serial.print(angle_found);
    Serial.print(" | Tilt Angle: ");
    Serial.println(tilt_x);
  }
}
