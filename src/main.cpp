#include <Arduino.h>
#include <SPI.h>
#include <TimerInterrupt_Generic.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <step.h>
float Kp=5000;
float Ki=0;
float Kd=0.01;
float accel = 0;
float absolutemax_accel = 30;
float absolutemax_speed = 50;
float setpoint = 0;
float tilt_x =0, tilt_x_prev=0;
float k=0;
#define LED 2



float value_speed = 9;
float motorspeed =0;
float tiltx_prev = 0.0;
float c = 0.85;
float gyro_y = 0;
float gyro_x_prev = 0;
float dt = 0, last_time=0, dg=0;
float tilt_var =0;
float now;
float integral, proportional, derivative, previous, output=0;



float error=0, error_prev1 =0, error_prev2=0,controller_prev1 =0, controller_output=0;

float az=0, gz_now=0, gz_prev=0, tilt_comp_prev =0, tilt_comp = 0;

// The Stepper pins
const int STEPPER1_DIR_PIN  = 16;
const int STEPPER1_STEP_PIN = 17;
const int STEPPER2_DIR_PIN  = 4;
const int STEPPER2_STEP_PIN = 14;
const int STEPPER_EN_PIN    = 15; 

//ADC pins
const int ADC_CS_PIN        = 5;
const int ADC_SCK_PIN       = 18;
const int ADC_MISO_PIN      = 19;
const int ADC_MOSI_PIN      = 23;

// Diagnostic pin for oscilloscope
const int TOGGLE_PIN        = 32;

const int PRINT_INTERVAL    = 500;
const int LOOP_INTERVAL     = 10;
const int STEPPER_INTERVAL_US = 20;

const float kx = 100.0;
const float VREF = 4.096;

//Global objects
ESP32Timer ITimer(3);
Adafruit_MPU6050 mpu;         //Default pins for I2C are SCL: IO22, SDA: IO21

step step1(STEPPER_INTERVAL_US,STEPPER1_STEP_PIN,STEPPER1_DIR_PIN );
step step2(STEPPER_INTERVAL_US,STEPPER2_STEP_PIN,STEPPER2_DIR_PIN );


//Interrupt Service Routine for motor update
//Note: ESP32 doesn't support floating point calculations in an ISR
bool TimerHandler(void * timerNo)
{
  static bool toggle = false;

  //Update the stepper motors
  step1.runStepper();
  step2.runStepper();

  //Indicate that the ISR is running
  digitalWrite(TOGGLE_PIN,toggle);  
  toggle = !toggle;
	return true;
}

uint16_t readADC(uint8_t channel) {
  uint8_t TX0 = 0x06 | (channel >> 2);  // Command Byte 0 = Start bit + single-ended mode + MSB of channel
  uint8_t tx1 = (channel & 0x03) << 6;  // Command Byte 1 = Remaining 2 bits of channel

  digitalWrite(ADC_CS_PIN, LOW); 

  SPI.transfer(TX0);                    // Send Command Byte 0
  uint8_t RX0 = SPI.transfer(tx1);      // Send Command Byte 1 and receive high byte of result
  uint8_t rx1 = SPI.transfer(0x00);     // Send dummy byte and receive low byte of result

  digitalWrite(ADC_CS_PIN, HIGH); 

  uint16_t result = ((RX0 & 0x0F) << 8) | rx1; // Combine high and low byte into 12-bit result
  return result;
}

void setup()
{
  pinMode(LED,OUTPUT);
  Serial.begin(115200);
  pinMode(TOGGLE_PIN,OUTPUT);

  // Try to initialize Accelerometer/Gyroscope
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  Serial.println("MPU6050 Found!");

  mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
  mpu.setGyroRange(MPU6050_RANGE_250_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_44_HZ);

  //Attach motor update ISR to timer to run every STEPPER_INTERVAL_US μs
  if (!ITimer.attachInterruptInterval(STEPPER_INTERVAL_US, TimerHandler)) {
    Serial.println("Failed to start stepper interrupt");
    while (1) delay(10);
    }
  Serial.println("Initialised Interrupt for Stepper");

  //Set motor acceleration values
  

  step1.setAccelerationRad(accel);
  step2.setAccelerationRad(accel);

  //Enable the stepper motor drivers
  pinMode(STEPPER_EN_PIN,OUTPUT);
  digitalWrite(STEPPER_EN_PIN, false);

  //Set up ADC and SPI
  pinMode(ADC_CS_PIN, OUTPUT);
  digitalWrite(ADC_CS_PIN, HIGH);
  SPI.begin(ADC_SCK_PIN, ADC_MISO_PIN, ADC_MOSI_PIN, ADC_CS_PIN);

}

float pid(float error){
  if (dt <= 0) return 0;

  float integral_max = 5;
  
  
  proportional = error;
  integral += integral * dt;
  integral = constrain(integral, -integral_max, integral_max);
  derivative = (error-previous) / dt;
  previous = error;
  output = (Kp * proportional) + (Ki * integral) + (Kd * derivative);

  if(output > absolutemax_accel)
    return absolutemax_accel;
  else if(output < -absolutemax_accel)
    return -absolutemax_accel;


   
  return output;
}

// float pid_1(float error1){
  
//   float A = Kp +(Ki * dt) + (Kd/dt);
//   float B = (-Kp - 2*(Kd/dt));
//   float C = (Kd/dt);

//   controller_output = controller_prev1 + (error1 * A) + (error_prev1 * B) + (error_prev2 * C);

//   error_prev2 = error_prev1;
//   error_prev1 = error1;
//   controller_prev1 = controller_output;

//  // if(controller_output > absolutemax)
//  //   return absolutemax;
//  // else if(controller_output < -absolutemax)
//  //return -absolutemax;

//   return controller_output;
// }


void loop()
{
  //Static variables are initialised once and then the value is remembered betweeen subsequent calls to this function
  static unsigned long printTimer = 0;  //time of the next print
  static unsigned long loopTimer = 0;   //time of the next control update
  static float tiltx = 0.0;             //current tilt angle
  
  //Run the control loop every LOOP_INTERVAL ms
  if (millis() > loopTimer) {
    loopTimer += LOOP_INTERVAL;
    now = millis();
    dt = (now - last_time)/1000;
    last_time = now;
    // Fetch data from MPU6050
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);



    
    // float accel_angle = atan2(a.acceleration.y, a.acceleration.z) * 180.0 / PI;
    float accel_angle = a.acceleration.z / 9.67;
    gyro_y = g.gyro.y; // degrees per second

    tilt_x = 0.98 * (tilt_x_prev + gyro_y * dt) + 0.02 * accel_angle;
    tilt_x_prev = tilt_x;



    error = (setpoint - (tilt_x));
    accel = pid(error);


    if(motorspeed > absolutemax_speed){
      motorspeed = absolutemax_speed;
    }
    else if(motorspeed < -absolutemax_speed){
      motorspeed = absolutemax_speed;
    }


    if(accel<0){
      motorspeed = -value_speed;
    }
    else{
      motorspeed = value_speed;
    }


    step1.setTargetSpeedRad(motorspeed);
    step2.setTargetSpeedRad(-motorspeed);

    step1.setAccelerationRad(accel);
    step2.setAccelerationRad(accel);

    k=k+1;
    if(k==701){
      setpoint = tilt_x;
      digitalWrite(LED,HIGH);
    }
    

  }
  
  //Print updates every PRINT_INTERVAL ms
  //Line format: X-axis tilt, Motor speed, A0 Voltage
  if (millis() > printTimer) {
    printTimer += PRINT_INTERVAL;
    Serial.print(step1.getSpeedRad());
    Serial.print(',');
    Serial.print(tilt_x);
    Serial.print(',');
    Serial.print(motorspeed);
    Serial.print(',');
    Serial.print(setpoint);
    Serial.print(',');
    Serial.print(accel);
    Serial.println();
  }
}