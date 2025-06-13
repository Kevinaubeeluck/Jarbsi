#include <Arduino.h>
#include <WiFi.h>
#include <cstring>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <SPI.h>



//

//ADC pins
const int ADC_CS_PIN        = 5;
const int ADC_SCK_PIN       = 18;
const int ADC_MISO_PIN      = 19;
const int ADC_MOSI_PIN      = 23;

int count = 0;
float POWER_INTERVAL = 50;
float V_M, I_M,V_5, I_5,V_B;
float CoulombCount = 0;
const float ratedCoulomb = 2*3600;
float I_mAvg = 0;
float I_5Avg = 0;
float VREF = 4.096;
long prev_power_time = 0;
//

// WiFi credentials
const char* ssid = "meow";
const char* password = "hello12345@";

// Server config
const char* server_ip = "192.168.84.234";
const int server_port = 12000;

// PID parameter
float Kp = 0.0;
float turn = 0;
float percentage = 77;

// TCP socket
int clientSocket = -1;

// Task handle for receiver loop
TaskHandle_t recvTaskHandle = NULL;

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

      if (sscanf(buffer, "SET_KP(%f)", &a) == 1) {
        Kp = a;
        Serial.print("Updated Kp: ");
        Serial.println(Kp);
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
      memset(buffer, 0, sizeof(buffer));
    }

    vTaskDelay(10 / portTICK_PERIOD_MS);
  }
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


  //Set up ADC and SPI
  pinMode(ADC_CS_PIN, OUTPUT);
  digitalWrite(ADC_CS_PIN, HIGH);
  SPI.begin(ADC_SCK_PIN, ADC_MISO_PIN, ADC_MOSI_PIN, ADC_CS_PIN);
}

void loop() {
  static unsigned long timer,powerTimer = 0;

  if (millis() > powerTimer) {
    
    powerTimer += POWER_INTERVAL;
    
    //Motor Current
    I_M = ((readADC(5) * VREF)/4095.0);    
    //I_M = I_M + 0.01;
    I_M = I_M - V_5/4.96;
    //I_M = I_M *1000; // convert to mV
    //I_M = I_M/1000; //divide differential gain
    //I_M = I_M - 0.1; // offset error 
    //I_M = I_M + 0.02;
    I_M = I_M*6.7;
    I_M = I_M/100; //divide resistance for current
    I_mAvg += I_M;
    count = count+1;
    
    //Motor Voltage
    V_M = ((readADC(2) * VREF)/4095.0); // Voltage divider circuit
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
    
    I_mAvg/=count;
    I_5Avg/=count;
    
    CoulombCount = CoulombCount + (I_mAvg + I_5Avg)*(millis()-prev_power_time)/1000; //rectangle approximation otherwise previous values must be saved
    

    timer += 1000;
    Serial.print("Current Kp: ");
    Serial.println(Kp);
    Serial.print("Current battery: ");
    Serial.println(percentage-(CoulombCount/ratedCoulomb)*100);
    Serial.print("Current turn: ");
    Serial.println(turn);


    if (clientSocket < 0) {                // reconnect if needed
      Socketconnect();
    }




    // Optionally send Kp value to server every second
    if (clientSocket > 0) {
      char bigbuffer[512];
      snprintf(bigbuffer, sizeof(bigbuffer),
              "Kp=%.2f\nBAT:%f\nBATVOLT:%f\n5VVOLT:%f\nMOTVOLT:%f\nMOTCURR:%f\n5VCURR:%f\n5V_Power:%f\nMotor_Power:%f\n",
              Kp,
              percentage-(CoulombCount/ratedCoulomb) * 100,
              V_B,
              V_5,
              V_M,
              I_mAvg,
              I_5Avg,
              V_5*I_5Avg,
              V_M*I_mAvg);
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