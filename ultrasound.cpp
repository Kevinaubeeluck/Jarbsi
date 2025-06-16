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

// Define TX and RX pins for UART (change if needed)
#define TXD1 17 
#define RXD1 16
#define trigPin 27
#define echoPin 26
bool blindspot = false;
void setup() {
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  Serial.begin(115200);
 }

void loop() {
  //Ultra sound distance measurement
  long duration, distance;
  digitalWrite(trigPin, LOW);  // Ensures its off before triggering
  delayMicroseconds(2); //slight delay
  digitalWrite(trigPin, HIGH); //activate trigger for 10us
  delayMicroseconds(10); 
  digitalWrite(trigPin, LOW); //end trigger
  duration = pulseIn(echoPin, HIGH); //activate echo pin which deactivates when a reflected signal is received and returns the duration that the pin is on for 
  distance = (duration)*0.0343/2; //multiply by speed of sound in cm/ms and divide 2 since it travels double the range
  
  if (distance <= 15 & distance >= 0){
    Serial.println("Obstacle within the blindspot");
    blindspot = true;
  }
  else {
    Serial.println("Out of range");
    blindspot = false;
  }


  
