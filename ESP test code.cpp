#include <Arduino.h>
#include <SPI.h>
#include <TimerInterrupt_Generic.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <step.h>
#include <WiFi.h>
#include <cstring>
#include <iostream>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include "MadgwickAHRS.h"

#define TXD1 26
#define RXD1 27
#define trigPin 27
#define echoPin 26
bool blindspot = false;

float integral, proportional, derivative, previous, output=0;
float absolutemax_accel = 65;
float absolutemax_speed = 35;
float tilt_x =0, tilt_x_prev=0;
float dt = 0, last_time=0;
float accel = 0;
float motorspeed =0;
float gyro_y = 0;
float now;
static unsigned long loopTimer = 0;
float error=0;
float angle_found=0.13;
float integral2=0, proportional2, derivative2;
float integral3=0, proportional3, derivative3;
float previous2;
float previous3;
float output2=0;
float output3 =0;
float now2 =0, dt2 =0, last_time2=0, outeraddon=0;
float motorspeed_setpoint =0;
float manual_override =0;
//pid values

float Kp=3000, Ki=0, Kd=0.04;
float Kmp= -0.002, Kmi = 0, Kmd = -0.00003;
float K3p = -0.5, K3i = 0, K3d = 0;


// WiFi credentials
const char* ssid = "ET";
const char* password = "mjcm6581";

// Server config
const char* server_ip = "192.168.92.234";
const int server_port = 12000;

// PID parameter
//float Kp = 0.0;
float turn = 0;
float percentage = 77;

// TCP socket
int clientSocket = -1;

 // power stuff
int count = 0;
float POWER_INTERVAL = 50;
float V_M, I_M,V_5, I_5,V_B;
float CoulombCount = 0;
const float ratedCoulomb = 2*3600;
float I_mAvg = 0;
float I_5Avg = 0;
float VREF = 4.096;
long prev_power_time = 0;

// Task handle for receiver loop
TaskHandle_t recvTaskHandle = NULL;

ESP32Timer ITimer(3);
Adafruit_MPU6050 mpu;  
Madgwick filter;

//yaw readings (angular rotation readings)
float rotationtimer=0;
float alpha = 0.9;  // Higher = smoother, lower = faster
float gx_bias = 0, gy_bias = 0, gz_bias = 0;
float yawRate_deg=0;
float turn_rate=0;

float want_speed = 0.13;
//float turn = 0;
float c = 1, d = 2, e = 3;
unsigned long lastUpdate = 0;
float loopTimer2=0;

float bias =0;

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

String Uartmessage;


float timepass1 = 0;
float timepass2 = 0;

step step1(STEPPER_INTERVAL_US,STEPPER1_STEP_PIN,STEPPER1_DIR_PIN );
step step2(STEPPER_INTERVAL_US,STEPPER2_STEP_PIN,STEPPER2_DIR_PIN );

float angle_found_robot =0;
float left_logic =0, right_logic=0;



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

void Wificonnect() {
  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi connected. IP:");
  Serial.println(WiFi.localIP());
}

void Socketconnect() {
  clientSocket = socket(AF_INET, SOCK_STREAM, 0);
  if (clientSocket < 0) {
    Serial.println("Socket creation failed.");
    return;
  }
  sockaddr_in serverAddress;
  serverAddress.sin_family = AF_INET;
  serverAddress.sin_port = htons(server_port);
  inet_pton(AF_INET, server_ip, &serverAddress.sin_addr);

  int connection = connect(clientSocket, (struct sockaddr*)&serverAddress, sizeof(serverAddress));
  if (connection == 0) {
    Serial.println("Connected to server!");
  } else {
    Serial.println("Failed to connect to server :(");
    Serial.print("errno: ");
    Serial.println(errno);
  }

}

float direction=0;
// Receiver loop task
void recvloopTask(void* pvParameters) {
  char buffer[256] = {0};
  float a;
  while (true) {
    int bytesReceived = recv(clientSocket, buffer, sizeof(buffer), 0);
    if (bytesReceived > 0) {
      buffer[bytesReceived] = '\0';
      //Serial.println(buffer);
      //Serial.println(percentage);
      
      if(manual_override == 1){

        if (sscanf(buffer, "SET_direction(%f)", &a) == 1) {
          direction = a;
          // Serial.print("Updated direction: ");
          // Serial.println(direction);
        }
        if (sscanf(buffer, "SET_TURN(%f)", &a) == 1) {
          turn = a;
          // Serial.print("Updated turn: ");
          // Serial.println(turn);
        }
        if (sscanf(buffer, "SET_DIR(%f)", &a) == 1) {
          direction = a;
          if(direction ==1){
            //Serial.print("direction: forward ");
          }
          if(direction == -1){
            //Serial.print("direction: backwards");
          }
          if(direction==0){
            //Serial.print("direction:still");
          }
        }

      }

      if(sscanf(buffer, "BAT:%f", &a) == 1){
        percentage = a;
        // Serial.println("Server percentage: ");
        // Serial.println(percentage);     
      }
      if (sscanf(buffer, "SET_KP(%f)", &a) == 1) {
        Kp = a;
        // Serial.print("Updated Kp: ");
        // Serial.println(Kp);
      }
      if (sscanf(buffer, "SET_KI(%f)", &a) == 1) {
        Ki = a;
        // Serial.print("Updated KI: ");
        // Serial.println(Ki);
      }
      if (sscanf(buffer, "SET_KD(%f)", &a) == 1) {
        Kd = a;
        // Serial.print("Updated KD: ");
        // Serial.println(Kd);
      }

      if (sscanf(buffer, "SET_K3P(%f)", &a) == 1) {
        K3p = a;
        // Serial.print("Updated K3p: ");
        // Serial.println(K3p);
      }
      if (sscanf(buffer, "SET_K3I(%f)", &a) == 1) {
        K3i = a;
        // Serial.print("Updated K3I: ");
        // Serial.println(K3i);
      }
      if (sscanf(buffer, "SET_K3D(%f)", &a) == 1) {
        K3d = a;
        // Serial.print("Updated K3D: ");
        // Serial.println(K3d);
      }
      if (sscanf(buffer, "SET_KMP(%f)", &a) == 1) {
        Kmp = a;
        // Serial.print("Updated Kmp: ");
        // Serial.println(Kmp);
      }
      if (sscanf(buffer, "SET_KMI(%f)", &a) == 1) {
        Kmi = a;
        // Serial.print("Updated KMI: ");
        // Serial.println(Kmi);
      }
      if (sscanf(buffer, "SET_KMD(%f)", &a) == 1) {
        Kmd = a;
        // Serial.print("Updated KMD: ");
        // Serial.println(Kmd);
      }
      if (sscanf(buffer, "bias(%f)", &a) == 1) {
        bias = a;
        // Serial.print("Updated bias: ");
        // Serial.println(bias);
      }
      if (sscanf(buffer, "MANUAL_OVERRIDE(%f)", &a) == 1) {
        manual_override = a;
        // Serial.print("manual override: ");
        // Serial.println(manual_override);
      }

    }
    memset(buffer, 0, sizeof(buffer));
    
    vTaskDelay(10 / portTICK_PERIOD_MS);
  }
}





void UARTrecv(String command) {
  // First, remove any leading/trailing whitespace or newline characters.
  // This makes the comparison much more reliable.
  command.trim();

  // Only process commands if the robot is in autonomous mode.
  if (manual_override == 0) {
    // Check if the command string is exactly "Forward"
    if (command == "Forward") {
      direction = 10;
      turn = 0;
      //Serial.println("Received command: Forward");

    } 
    // Otherwise, check if it's "Right"
    else if (command == "Right") {
      direction = 0;
      turn = -13;
      //Serial.println("Received command: Right");

    } 
    // Otherwise, check if it's "Left"
    else if (command == "Left") {
      direction = 0;
      turn = 13;

      //Serial.println("Received command: Left");
    }
  }
}


float pid_1(float error){
  if (dt <= 0) return 0;

  float integral_max = 5;
  
  proportional = error;
  integral += error * dt;
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

float pid_2(float error2, float dt2){
  if (dt2 <= 0) return 0;
  float integral_max = 2;  
  proportional2 = error2;
  integral2 += error2 * dt2;
  integral2 = constrain(-integral, integral_max, integral_max);
  derivative2 = (error2-previous2) / dt2;
  previous2 = error2;
  output2 = (Kmp * proportional2) + (Kmi * integral2) + (Kmd * derivative2);
  return output2;
}

float pid_3 (float error3, float dt3){

  if (dt3 <= 0) return 0;
  float integral_max = 5;  
  proportional3 = error3;
  integral3 += error3 * dt3;
  integral3 = constrain(integral3, -integral_max, integral_max);
  derivative3 = (error3-previous3) / dt3;
  previous3 = error3;
  output3 = (K3p * proportional3) + (K3i * integral3) + (K3d * derivative3);
  return output3;

}

void setup() {
  Serial.begin(115200);  // USB Serial
  Serial1.begin(230400, SERIAL_8N1, RXD1, TXD1);  // UART1
  delay(1000);

  Wificonnect();
  Socketconnect();

  xTaskCreatePinnedToCore(
    recvloopTask,
    "RecvLoopTask",
    4096,
    NULL,
    1,
    &recvTaskHandle,
    1
  );

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

  filter.begin(100);  // 100 Hz
  calibrateGyro();


  //Attach motor update ISR to timer to run every STEPPER_INTERVAL_US Î¼s
  if (!ITimer.attachInterruptInterval(STEPPER_INTERVAL_US, TimerHandler)) {
    Serial.println("Failed to start stepper interrupt");
    while (1) delay(10);
    }
  Serial.println("Initialised Interrupt for Stepper");



  step1.setAccelerationRad(accel);
  step2.setAccelerationRad(accel);

  //Enable the stepper motor drivers
  pinMode(STEPPER_EN_PIN,OUTPUT);
  digitalWrite(STEPPER_EN_PIN, false);

  //Set up ADC and SPI
  pinMode(ADC_CS_PIN, OUTPUT);
  digitalWrite(ADC_CS_PIN, HIGH);
  SPI.begin(ADC_SCK_PIN, ADC_MISO_PIN, ADC_MOSI_PIN, ADC_CS_PIN);

  //Serial.println("hello from setup");

}
float n=0;
float m=0;


void loop() {
  static unsigned long timer = 0; //server updating timer
  static unsigned long loopTimer = 0;
  static unsigned long loopTimer2 = 0;
  static unsigned long loopTimer3 = 0;
  static unsigned long loopTimer4 = 0;
  static unsigned long powerTimer = 0;
  

  //Serial.println("hello from loopers");

  String Uartmessage;
  
  
  
  // if (millis() > loopTimer4) {
  //   loopTimer4 += 500;
    if (Serial1.available()) {
      Uartmessage = Serial1.readStringUntil('\n');
      Serial.println("Received: " + Uartmessage);
      UARTrecv(Uartmessage);
    }
  // }

  // outer loop at 10hz for velocity control.
  if (millis() > loopTimer2) {
    loopTimer2 += 100;
    now2 = millis();
    dt2 = (now2 - last_time2)/1000;
    last_time2 = now2;

    outeraddon = pid_2(direction-(((step1.getSpeedRad() - step2.getSpeedRad())/2)),dt2);
    outeraddon = constrain(outeraddon, -0.03, 0.03);

      
    


  }



  
  if (millis() > powerTimer) {
    
    powerTimer += POWER_INTERVAL;
    
    //Motor Current
    I_M = ((readADC(5) * VREF)/4095.0);    
    //I_M = I_M + 0.01;
    I_M = I_M - V_5/4.96;
    I_M = I_M *1000; // convert to mV
    I_M = I_M/10; //divide differential gain
    //I_M = I_M - 0.1; // offset error 
    //I_M = I_M + 0.02;
    I_M = I_M*6.7;
    I_M = I_M/100; //divide resistance for current
    I_mAvg += I_M;
    count = count+1;
    
    //Motor Voltage
    V_M = ((readADC(1) * VREF)/4095.0); // Voltage divider circuit
    V_M = V_M+0.03;//quantization error + error from connecting to pin
    V_M = V_M - 0.013;
    V_M = V_M* 6.15;

    //5V current
    I_5 = ((readADC(3) * VREF)/4095.0) ; //this time only gain 100
    I_5 = I_5+ 0.03; // quant error
    I_5 = I_5 - V_5/4.96; // reference voltage get actual differential
    I_5 = I_5*1000; //convert to mV
    I_5 = I_5/100; // divide differential gain
    I_5 = I_5 -0.1; // offset error

    I_5 = I_5/10; // convert to current value
    I_5Avg += I_5;

    //Battery Voltage
    V_B = ((readADC(4) * VREF)/4095.0)+0.034; //0.03 quantization error
    V_B = V_B - 0.02; //input offset voltage
    V_B = V_B *6.17; //voltage divider
    
    //5V
    V_5 = ((readADC(0) * VREF)/4095.0); 
    V_5 = V_5 + 0.017;// quantization error
    V_5 = V_5 -0.007; //input offset voltage
    V_5 = V_5*4.96; //Voltage divider
    //Serial.println(V_5);
    //CouloumbCount = CouloumbCount + (I_M + I_5)*POWER_INTERVAL/1000; //rectangle approximation otherwise previous values must be saved

  }





  if (millis() > timer) {

    //averaging values for power monitoring:
    I_mAvg/=count;
    I_5Avg/=count;
    
    CoulombCount = CoulombCount + (I_mAvg + I_5Avg)*(millis()-prev_power_time)/1000; //rectangle approximation otherwise previous values must be saved
    


    timer += 1000;
    Serial.print("manual override on:");
    Serial.println(manual_override);
    // Serial.print(" ,speed:");
    // Serial.print(direction);
    // Serial.print(" ,turn:");
    // Serial.print(turn);
    // Serial.print(" ,Uart message:");
    // Serial.print(Uartmessage);
    // Serial.print(" ,K3p:");
    // Serial.print(K3p);
    // Serial.print(" ,K3i:");
    // Serial.print(K3i);
    // Serial.print(" ,K3d:");
    // Serial.println(K3d);

  

    if (clientSocket < 0) {                // reconnect if needed
      Socketconnect();
    } 

    // Optionally send Kp value to server every second
     if (clientSocket > 0) {
      char bigbuffer[512];
      snprintf(bigbuffer, sizeof(bigbuffer),
              "turn:%.2f\ndir:%f\nuart:%f\n",
              turn,
              direction,
              Uartmessage);




              // "Kp=%.2f\nBAT:%f\nBATVOLT:%f\n5VVOLT:%f\nMOTVOLT:%f\nMOTCURR:%f\n5VCURR:%f\n5V_Power:%f\nMotor_Power:%f\n",
              // Kp,
              // percentage-(CoulombCount/ratedCoulomb) * 100,
              // V_B,
              // V_5,
              // V_M,
              // I_mAvg,
              // I_5Avg,
              // V_5*I_5Avg,
              // V_M*I_mAvg);
      //percentage -= 0.01;

      //Serial.print(bigbuffer);
      send(clientSocket, bigbuffer, strlen(bigbuffer), 0);


    }
    I_mAvg = 0;
    I_5Avg = 0;
    count = 0;
    prev_power_time = millis();
  }

  if (millis() > loopTimer) {
    loopTimer += LOOP_INTERVAL;
    now = millis();
    dt = (now - last_time)/1000;
    last_time = now;
    // Fetch data from MPU6050
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);



    //tilt angle calculations
    float accel_angle = a.acceleration.z / 9.67;
    gyro_y = g.gyro.y; // degrees per second
    tilt_x = 0.98 * (tilt_x_prev + gyro_y * dt) + 0.02 * accel_angle;
    tilt_x_prev = tilt_x;


    //yaw angle calculations
    filter.updateIMU(
      g.gyro.x, g.gyro.y, g.gyro.z,
      a.acceleration.x, a.acceleration.y, a.acceleration.z
    );
    float q0 = filter.q0;
    float q1 = filter.q1;
    float q2 = filter.q2;
    float q3 = filter.q3;
    float gx = g.gyro.x - gx_bias;
    float gy = g.gyro.y - gy_bias;
    float gz = g.gyro.z - gz_bias;

    float gw_z = 2*(q1*q3 - q0*q2)*gx + 2*(q2*q3 + q0*q1)*gy + (1 - 2*q1*q1 - 2*q2*q2)*gz;
    yawRate_deg = gw_z * 180.0 / PI;





    //angle tilt pid loop
    error =   bias - (tilt_x) + outeraddon;
    //error = bias - tilt_x;
    accel = pid_1(error);
    turn_rate = pid_3(turn - yawRate_deg, dt);
    //turn_rate = 0;


    //motorspeed clamping and control
    motorspeed += accel * dt;
    if(motorspeed > 20){
      motorspeed = 20;
    }
    else if (motorspeed < -20){
      motorspeed = -20;
    }
    
    // if(accel>0){
    //   motorspeed = 20;
    // }
    // else if (accel < 0){
    //   motorspeed = -20;
    // }

// Clamp it if needed


    // if(tilt_x < bias-0.15|| tilt_x > bias + 0.15){
    //   step1.setTargetSpeedRad(0);
    //   step2.setTargetSpeedRad(0);
    // }

    float accel1 = accel + turn_rate;
    float accel2 = accel - turn_rate;
    step1.setTargetSpeedRad(motorspeed);
    step2.setTargetSpeedRad(-motorspeed);
    step1.setAccelerationRad(accel1);
    step2.setAccelerationRad(accel2);
  }
}



