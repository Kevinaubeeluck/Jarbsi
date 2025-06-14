#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include "MadgwickAHRS.h"

Adafruit_MPU6050 mpu;
Madgwick filter;

float yawRateFiltered = 0;
float alpha = 0.9;  // Higher = smoother, lower = faster
float gx_bias = 0, gy_bias = 0, gz_bias = 0;

void calibrateGyro() {
  const int samples = 200;
  for (int i = 0; i < samples; i++) {
    sensors_event_t a, g, t;
    mpu.getEvent(&a, &g, &t);
    gx_bias += g.gyro.x;
    gy_bias += g.gyro.y;
    gz_bias += g.gyro.z;
    delay(5);
  }
  gx_bias /= samples;
  gy_bias /= samples;
  gz_bias /= samples;
}

float rotationtimer=0;

void setup() {
  Serial.begin(115200);
  if (!mpu.begin()) {
    Serial.println("MPU not found!");
    while (1);
  }

  

  mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
  mpu.setGyroRange(MPU6050_RANGE_250_DEG);
  filter.begin(100);  // 100 Hz

  calibrateGyro();
}




void loop() {
  static unsigned long lastUpdate = 0;
  float dt = (millis() - lastUpdate) / 1000.0;
  lastUpdate = millis();

  sensors_event_t a, g, t;
  mpu.getEvent(&a, &g, &t);

  // Feed data to Madgwick filter
  filter.updateIMU(
    g.gyro.x, g.gyro.y, g.gyro.z,
    a.acceleration.x, a.acceleration.y, a.acceleration.z
  );

  // Extract quaternion
  float q0 = filter.q0;
  float q1 = filter.q1;
  float q2 = filter.q2;
  float q3 = filter.q3;

  // Rotate gyro vector to world frame
  float gx = g.gyro.x - gx_bias;
  float gy = g.gyro.y - gy_bias;
  float gz = g.gyro.z - gz_bias;

  float gw_z = 2*(q1*q3 - q0*q2)*gx + 2*(q2*q3 + q0*q1)*gy + (1 - 2*q1*q1 - 2*q2*q2)*gz;
  float yawRate_deg = gw_z * 180.0 / PI;




  //yawRateFiltered = alpha * yawRateFiltered + (1 - alpha) * yawRate_deg;

  Serial.print("Yaw rate (deg/s): ");
  Serial.println(yawRate_deg, 2);

  unsigned long now = millis();
  Serial.println(1000.0 / (now - lastUpdate));
  lastUpdate = now;

  delay(10);

    
  if (millis() > rotationtimer) {
    rotationtimer += 1000;  
    Serial.print(1000.0 / (now - lastUpdate));
    Serial.print(  "Yaw rate (deg/s): ");
    Serial.println(yawRate_deg, 2);

  }
}
