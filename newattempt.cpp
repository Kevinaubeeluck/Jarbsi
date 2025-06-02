#include <Arduino.h>
#include <SPI.h>
#include <TimerInterrupt_Generic.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <step.h>
#include <cstring>
#include <iostream>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>
#include <arpa/inet.h>
#include <stdlib.h>
#include <WiFi.h> 
#include <regex>

using namespace std;

//------------------------------------------------------------------------
// global floats

// variable floats
float Kp = 3000, Ki = 0, Kd = 0.01;
float Kpv = 1, Kiv = 0, Kdv = 0.1;

float max_accel_int = 2, min_accel_int = 2;
float max_accel = 75, min_accel = -75;

float maxivel = 2, minivel =-2;
float maxtilt = 0.04, mintilt = -0.04;

float motormax = 35, motormin = -35;




// initialised only
float dt=0, last_time=0, now;
float prop_accel =0, integral_accel =0, derivative_accel =0, prev_err_accel =0;
float prop_velocity =0, integral_velocity =0, derivative_velocity =0, prev_err_velocity =0;
float tilt_x=0, tilt_x_prev=0;

float velocity=0, velocity_prev =0, error_velocity, angle_found, setvelocity=0;
float motorspeed, accel;



//constant floats-----------
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

const char* ssid = "meow";
const char* password = "hello12345@";

//-------------------------------------------------------------------------

// Global client socket for loop 
int clientSocket = -1;

//Global objects
ESP32Timer ITimer(3);
Adafruit_MPU6050 mpu;         //Default pins for I2C are SCL: IO22, SDA: IO21

step step1(STEPPER_INTERVAL_US,STEPPER1_STEP_PIN,STEPPER1_DIR_PIN );
step step2(STEPPER_INTERVAL_US,STEPPER2_STEP_PIN,STEPPER2_DIR_PIN );

TaskHandle_t recvTaskHandle = NULL;

//Interrupt Service Routine for motor update
//Note: ESP32 doesn't support floating point calculations in an ISR
//-----------------------------------------------------------------------------


void recvloopTask(void* pvParameters){
  char buffer[1024] = {0};
  //regex floatTupleRegex(R"(\(\s*-?\d*\.?\d+\s*,\s*-?\d*\.?\d+\s*,\s*-?\d*\.?\d+\s*\))");
  while (true) {
      int bytesReceived = recv(clientSocket, buffer, sizeof(buffer), 0);
      //if (regex_match(buffer, floatTupleRegex)) { 
      float a;
      buffer[bytesReceived] = '\0'; // null-terminate the string

    // if (sscanf(buffer, "%f,%f,%f,%f,%f,%f,%f,%f,%f", &a, &b, &c, &d, &e, &f, &g, &h, &i) == 9) {
    //     Kp = a;
    //     Ki = b;
    //     Kd = c;
    //     Kpv = d;
    //     Kiv = e;
    //     Kdv = f;
    //     maxtilt = g;
    //     mintilt = h;
    //     setvelocity = i;
    //           //cout << "Parsed values: Kp=" << Kp << ", Ki=" << Ki << ", Kd=" << Kd << endl;
    // }

    if (sscanf(buffer, "SET_KP(%f)", &a) == 1) {
        Kp = a;
    }
    else if (sscanf(buffer, "SET_KI(%f)", &a) == 1) {
        Ki = a;
    }
    else if (sscanf(buffer, "SET_KD(%f)", &a) == 1) {
        Kd = a;
    }
    else if (sscanf(buffer, "SET_Kpv(%f)", &a) == 1) {
        Kpv = a;
    }
    else if (sscanf(buffer, "SET_Kiv(%f)", &a) == 1) {
        Kiv = a;
    }
    else if (sscanf(buffer, "SET_Kdv(%f)", &a) == 1) {
        Kdv = a;
    }
    else if (sscanf(buffer, "MAX_TILT(%f)", &a) == 1) {
        maxtilt = a;
    }
    else if (sscanf(buffer, "MIN_TILT(%f)", &a) == 1) {
        mintilt = a;
    }
    else if (sscanf(buffer, "TARGET_SPEED(%f)", &a) == 1) {
        setvelocity = a;
    }

      //}
      memset(buffer, 0, sizeof(buffer));
      vTaskDelay(10 / portTICK_PERIOD_MS);
  }
      
}


void Wificonnect(){
  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi connected. IP: ");
  Serial.println(WiFi.localIP());
}

void Socketconnect(){
  // Connect socket
  clientSocket = socket(AF_INET, SOCK_STREAM, 0);

  // specifying address
  sockaddr_in serverAddress;
  serverAddress.sin_family = AF_INET;
  serverAddress.sin_port = htons(12000);
  inet_pton(AF_INET, "192.168.219.158", &serverAddress.sin_addr);

  
  // sending connection request
  int connection = connect(clientSocket, (struct sockaddr*)&serverAddress,
          sizeof(serverAddress));
  
  if(connection == 0){
    Serial.println("HELL YEAH IT'S CONNECTED TO THE SERVER");
  }
  else{
    Serial.println(":( check wifi");
  }
}


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

  //Attach motor update ISR to timer to run every STEPPER_INTERVAL_US Î¼s
  if (!ITimer.attachInterruptInterval(STEPPER_INTERVAL_US, TimerHandler)) {
    Serial.println("Failed to start stepper interrupt");
    while (1) delay(10);
    }
  Serial.println("Initialised Interrupt for Stepper");

  pinMode(STEPPER_EN_PIN,OUTPUT);
  digitalWrite(STEPPER_EN_PIN, false);

  //Set up ADC and SPI
  pinMode(ADC_CS_PIN, OUTPUT);
  digitalWrite(ADC_CS_PIN, HIGH);
  SPI.begin(ADC_SCK_PIN, ADC_MISO_PIN, ADC_MOSI_PIN, ADC_CS_PIN);

  Wificonnect();

  Socketconnect();

  // thread reciever(recvloop,clientSocket);

xTaskCreatePinnedToCore(
  recvloopTask,        // Function
  "RecvLoopTask",      // Name
  4096,                // Stack size
  NULL,                // Params
  1,                   // Priority
  &recvTaskHandle,     // Task handle
  1                    // Core 1 (leave loop() on Core 0)
);  
    
}

  
float pid_accel(float error_accel){

    if (dt <= 0) return 0;
    prop_accel = error_accel;
    integral_accel += error_accel*dt;
    integral_accel = constrain(integral_accel, max_accel_int, min_accel_int);
    derivative_accel = (error_accel - prev_err_accel)/dt;
    prev_err_accel = error_accel;
    float output = (Kp * prop_accel) + (Ki * integral_accel) + (Kd * derivative_accel);

    output = constrain(output, max_accel, min_accel);

    return output;

}

float pid_velocity(float error_velocity){

    if (dt <= 0) return 0;

    prop_velocity = error_velocity;
    integral_velocity += error_velocity * dt;
    integral_velocity = constrain(integral_velocity, maxivel, minivel);
    derivative_velocity = (error_velocity - prev_err_velocity)* dt;
    prev_err_velocity = error_velocity;
    float output = (Kpv * prop_velocity) + (Kiv * integral_velocity) + (Kdv * derivative_velocity);
    output = constrain(output, maxtilt, mintilt);
    return output;
}




void loop(){


  static unsigned long printTimer = 0;  //time of the next print
  static unsigned long loopTimer = 0;   //time of the next control update
  float tiltx = 0.0;             //current tilt angle
  
  //Run the control loop every LOOP_INTERVAL ms
  if (millis() > loopTimer) {
    loopTimer += LOOP_INTERVAL;
    now = millis();
    dt = (now - last_time)/1000;
    last_time = now;
    // Fetch data from MPU6050
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);





    // second controller (velocity -> tilt angle)
    velocity_prev = velocity;
    velocity = step1.getSpeedRad();
    error_velocity = setvelocity - velocity;
    angle_found = pid_velocity(error_velocity);



    //get values from mpu

    float accel_angle = a.acceleration.z/9.67;
    float gyro_y = g.gyro.y;

    tilt_x = 0.98 *(tilt_x_prev + gyro_y * dt) + 0.02 * accel_angle;
    tilt_x_prev = tilt_x;

    //first controller (tilt angle -> acceleration)

    float error_a = angle_found - tilt_x;
    accel = pid_accel(error_a);

    //set speed and acceleration
    motorspeed += accel * dt;
    motorspeed = constrain(motorspeed, motormax, motormin);

    step1.setTargetSpeed(motorspeed);
    step2.setTargetSpeed(motorspeed);

    step1.setAccelerationRad(accel);
    step1.setAccelerationRad(accel);
  }

  if (millis() > printTimer) {
    printTimer += PRINT_INTERVAL;
    // Serial.print("Parsed values: Kp=");
    // Serial.print(Kp);
    // Serial.print(", Ki=");
    // Serial.print(Ki);
    // Serial.print(", Kd=");
    // Serial.print(Kd);
    // Serial.print(", Kpv=");
    // Serial.print(Kpv);
    // Serial.print(", Kiv=");
    // Serial.print(Kiv);
    // Serial.print(", Kdv=");
    // Serial.print(Kdv);
    // Serial.print(", max_tilt=");
    // Serial.print(maxtilt);
    // Serial.print(", min_tilt=");
    // Serial.print(mintilt);
    // Serial.print(", speed_setpoint=");
    // Serial.println(setvelocity);
    char buffer[100];
    snprintf(buffer, sizeof(buffer), "%.3f,%.3f,%.3f,%.3f,%.4f",step1.getSpeedRad(),tilt_x,motorspeed,accel);
    send(clientSocket, buffer, strlen(buffer), 0);
    memset(buffer, 0, 100);
    snprintf(buffer, sizeof(buffer), "%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f",Kp,Ki,Kd,Kpv,Kiv,Kdv,maxtilt,mintilt,setvelocity);
    send(clientSocket, buffer, strlen(buffer), 0);
    Serial.println();


  }  




}


