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
#include <string>      // for std::string
#include <cstring>  

// WiFi credentials
const char* ssid = "m123";
const char* password = "12345";

// Server config
const char* server_ip = "192.168.34.158";
const int server_port = 12000;

// PID parameter
//float Kp = 0.0;
float turn = 0;

float percentage = 100;

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

float want_speed = 0.13;
float target_yaw_rate = 0;
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
float target_speed = 0;

step step1(STEPPER_INTERVAL_US,STEPPER1_STEP_PIN,STEPPER1_DIR_PIN );
step step2(STEPPER_INTERVAL_US,STEPPER2_STEP_PIN,STEPPER2_DIR_PIN );

bool TimerHandler(oid * timerNo)
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

      
      if(sscanf(buffer, "BAT:%f", &a) == 1){
        percentage = a;
        Serial.println("Server percentage: ");
        Serial.println(percentage);     
      }
      
      }
      memset(buffer, 0, sizeof(buffer));
    }
    vTaskDelay(10 / portTICK_PERIOD_MS);
  }
}





void loop() {
  static unsigned long timer,powerTimer = 0;

  if (millis() > powerTimer) {
    
    powerTimer += POWER_INTERVAL;
    
    //Motor Current
    I_M = ((readADC(5) * VREF)/4095.0);    
    I_M = I_M - V_5/4.96;
    I_M = I_M * 1000; // convert to mV
    I_M = I_M/10; //divide differential gain
    I_M = I_M*6.7; //voltage divider
    I_M = I_M/100; //divide resistance for current
    I_mAvg += I_M;
    
    
    //Motor Voltage
    V_M = ((readADC(1) * VREF)/4095.0); // Voltage divider circuit
    V_M = V_M +0.03;//quantization error + error from connecting to pin
    V_M = V_M - 0.013; //offset error
    V_M = V_M* 6.15; //voltage divider

    //5V current
    I_5 = ((readADC(3) * VREF)/4095.0) ; //this time only gain 100
    I_5 = I_5+ 0.03; // quant error
    I_5 = I_5 - V_5/4.96; // reference voltage get actual differential
    I_5 = I_5*1000; //convert to mV
    I_5 = I_5/100; // divide differential gain
    I_5 = I_5 -0.1; // offset error
    I_5 = I_5/10; // convert to current value
    I_5Avg += I_5; // sum to take an average

    //Battery Voltage
    V_B = ((readADC(4) * VREF)/4095.0);
    V_B = V_B+0.034; //0.03 quantization error
    V_B = V_B - 0.02; //input offset voltage
    V_B = V_B *6.17; //voltage divider
    //5V
    V_5 = ((readADC(0) * VREF)/4095.0); 
    V_5 = V_5 + 0.017;// quantization error
    V_5 = V_5 -0.007; //input offset voltage
    V_5 = V_5*4.96; //Voltage divider

    count = count+1;
  }


  //server update and coulomb counting
  if (millis() > timer) {
    timer += 1000;

    I_mAvg/=count;
    I_5Avg/=count;
    
    CoulombCount = CoulombCount + (I_mAvg + I_5Avg)*(millis()-prev_power_time)/1000; //rectangle approximation
    
    if (clientSocket < 0) {                // reconnect if needed
      Socketconnect();
    }


    //Send Power monitoring data to server
    if (clientSocket > 0) {
      char bigbuffer[512];
      snprintf(bigbuffer, sizeof(bigbuffer),
              "\nBAT:%f\nBATVOLT:%f\n5VVOLT:%f\nMOTVOLT:%f\nMOTCURR:%f\n5VCURR:%f\n5V_Power:%f\nMotor_Power:%f\n",
              percentage-(CoulombCount/ratedCoulomb) * 100,
              V_B,
              V_5,
              V_M,
              I_mAvg,
              I_5Avg,
              V_5*I_5Avg, //power consumption of 5V supply P = IV
              V_M*I_mAvg);//power consumption of Motor supply P = IV
      //percentage -= 0.01;

      Serial.print(bigbuffer);
      send(clientSocket, bigbuffer, strlen(bigbuffer), 0);


    }
    I_mAvg = 0;
    I_5Avg = 0;
    count = 0;
    prev_power_time = millis();
  }










}
