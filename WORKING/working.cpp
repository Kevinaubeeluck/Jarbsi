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



float integral, proportional, derivative, previous, output=0;
float Kp=3000;
float Ki=0;
float Kd=0.04;
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

float integral2, proportional2, derivative2;
float Kmp= -0.002, Kmi = 0, Kmd = -0.00003;
float previous2;
float output2=0;
float now2 =0, dt2 =0, last_time2=0, outeraddon=0;
float motorspeed_setpoint =0;

// WiFi credentials
const char* ssid = "sachinator13";
const char* password = "hahahaha";

// Server config
const char* server_ip = "192.168.34.158";
const int server_port = 12000;

// PID parameter
//float Kp = 0.0;
float turn = 0;
float percentage = 77;

// TCP socket
int clientSocket = -1;

// Task handle for receiver loop
TaskHandle_t recvTaskHandle = NULL;

ESP32Timer ITimer(3);
Adafruit_MPU6050 mpu;  

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

float timepass1 = 0;
float timepass2 = 0;

step step1(STEPPER_INTERVAL_US,STEPPER1_STEP_PIN,STEPPER1_DIR_PIN );
step step2(STEPPER_INTERVAL_US,STEPPER2_STEP_PIN,STEPPER2_DIR_PIN );

float angle_found_robot =0;



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
      Serial.println(buffer);
      Serial.println(percentage);

      if (sscanf(buffer, "SET_direction(%f)", &a) == 1) {
        direction = a;
        Serial.print("Updated direction: ");
        Serial.println(direction);
      }
      if(sscanf(buffer, "BAT:%f", &a) == 1){
        percentage = a;
        Serial.println("Server percentage: ");
        Serial.println(percentage);     
      }
      if (sscanf(buffer, "SET_TURN(%f)", &a) == 1) {
        turn = a;
        Serial.print("Updated turn: ");
        Serial.println(turn);
      }
      if (sscanf(buffer, "bias(%f)", &a) == 1) {
        bias = a;
        Serial.print("Updated bias: ");
        Serial.println(bias);
      }
      if (sscanf(buffer, "SET_DIR(%f)", &a) == 1) {
        direction = a;
        if(direction ==1){
          Serial.print("direction: forward ");
        }
        if(direction == -1){
          Serial.print("direction: backwards");
        }
        if(direction==0){
          Serial.print("direction:still");
        }
      }
      memset(buffer, 0, sizeof(buffer));
    }
    vTaskDelay(10 / portTICK_PERIOD_MS);
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

  float integral_max = 5;  
  proportional2 = error2;
  integral2 += error2 * dt2;
  //integral2 = constrain(integral, -integral_max, integral_max);
  derivative2 = (error2-previous2) / dt2;
  previous2 = error2;
  output2 = (Kmp * proportional2) + (Kmi * integral2) + (Kmd * derivative2);
  return output2;

}

void setup() {
  Serial.begin(115200);
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

    
}
float n=0;
float m=0;


void loop() {



  if (millis() > loopTimer2) {
    loopTimer2 += 100;
    now2 = millis();
    dt2 = (now2 - last_time2)/1000;
    last_time2 = now2;

    outeraddon = pid_2(direction-(((step1.getSpeedRad() - step2.getSpeedRad())/2)),dt2);
    outeraddon = constrain(outeraddon, -0.03, 0.03);
  }


  static unsigned long timer = 0;
  static unsigned long loopTimer = 0;
  static unsigned long loopTimer2 = 0;
  if (millis() > timer) {
    timer += 1000;
    Serial.print("tilt:");
    Serial.print(tilt_x);
    Serial.print(" ,m:");
    Serial.print(m);
    Serial.print(" ,n:");
    Serial.print(n);
    Serial.print(" ,tilt_setpoint:");
    Serial.print(angle_found_robot);
    Serial.print(" ,motorspeed:");
    Serial.print(step1.getSpeedRad());
    Serial.print(" ,bias:");
    Serial.println(bias);
    Serial.print(" ,turn:");
    Serial.println(turn);

    if (clientSocket < 0) {                // reconnect if needed
      Socketconnect();
    }

    // Optionally send Kp value to server every second
    if (clientSocket > 0) {
      char bigbuffer[512];
      snprintf(bigbuffer, sizeof(bigbuffer),
              "Kp=%.2f\nBAT:%f\nBATVOLT:%f\n5VVOLT:%f\nMOTVOLT:%f\nMOTCURR:%f\n5VCURR:%f\n",
              Kp,
              percentage,
              percentage - 0.01,
              percentage - 0.02,
              percentage - 0.03,
              percentage - 0.04,
              percentage - 0.05);

      percentage -= 1;        
      send(clientSocket, bigbuffer, strlen(bigbuffer), 0);
    }
  }

  if (millis() > loopTimer) {
    loopTimer += LOOP_INTERVAL;
    now = millis();
    dt = (now - last_time)/1000;
    last_time = now;
    // Fetch data from MPU6050
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);



    float accel_angle = a.acceleration.z / 9.67;
    gyro_y = g.gyro.y; // degrees per second

    tilt_x = 0.98 * (tilt_x_prev + gyro_y * dt) + 0.02 * accel_angle;
    tilt_x_prev = tilt_x;



    error =   bias - (tilt_x) + outeraddon;
    accel = pid_1(error);

   //motorspeed += accel * dt;

    // if(motorspeed > 7){
    //   motorspeed = 7;
    // }
    // else if (motorspeed < -7){
    //   motorspeed = -7;
    // }
    
    if(accel>0){
      motorspeed = 20;
    }
    else if (accel < 0){
      motorspeed = -20;
    }

// Clamp it if needed


    float accel1 = accel;
    float accel2 = accel;


    
    step1.setTargetSpeedRad(motorspeed);
    step2.setTargetSpeedRad(-motorspeed);

    step1.setAccelerationRad(accel1+turn);
    step2.setAccelerationRad(accel2-turn);
  }
}