#include <Arduino.h>
#include <WiFi.h>
#include <cstring>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>

// WiFi credentials
const char* ssid = "meow";
const char* password = "hello12345@";

// Server config
const char* server_ip = "192.168.17.234";
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
}

void loop() {
  static unsigned long timer = 0;
  if (millis() > timer) {
    timer += 1000;
    Serial.print("Current Kp: ");
    Serial.println(Kp);
    Serial.print("Current battery: ");
    Serial.println(percentage);
    Serial.print("Current turn: ");
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

      send(clientSocket, bigbuffer, strlen(bigbuffer), 0);


    }
  }
}