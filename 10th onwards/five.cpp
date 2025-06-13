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
#include <WebServer.h>


using namespace std;
float outeraddon =0;
float Kp=3000;
float Ki=0;
float Kd=0.04;
float accel = 0;
float absolutemax_accel = 65;
float absolutemax_speed = 35;
float setpoint = 0;
float tilt_x =0, tilt_x_prev=0;
float k=0;
#define LED 2

float actual_motor_speed =0;
float actual_motor_speed_prev=0;
float motorspeed_setpoint=0;
float Kmp= 0.002, Kmi = 0, Kmd =0.00003;
float angle_found =0;
float error_speed =0;
float absolutemax_tilt = 0.15;
float absolutemin_tilt= 0.13;
float absolutetilit_error = 0.03;
float previous2;
float setpoint_set=0;
float output2=0;
float preve =0;



float value_speed = 35;
float motorspeed =0;
float tiltx_prev = 0.0;
float gyro_y = 0;
float gyro_x_prev = 0;
float dt = 0, last_time=0, dg=0, last_time2=0;
float tilt_var =0;
float now, now2;
float integral, proportional, derivative, previous, output=0;
float integral2, proportional2, derivative2;




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


const char* ssid = "sachinator13";
const char* password = "hahahaha";

//Global objects
ESP32Timer ITimer(3);
Adafruit_MPU6050 mpu;         //Default pins for I2C are SCL: IO22, SDA: IO21

WebServer server(80);
float want_speed = 0.14;
float turn = 0;
float c = 1, d = 2, e = 3;
unsigned long lastUpdate = 0;
step step1(STEPPER_INTERVAL_US,STEPPER1_STEP_PIN,STEPPER1_DIR_PIN );
step step2(STEPPER_INTERVAL_US,STEPPER2_STEP_PIN,STEPPER2_DIR_PIN );


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


void handleRoot() {
  String html = R"rawliteral(
    <!DOCTYPE html>
    <html>
    <head>
      <title>ESP32 Keyboard + Live Display</title>
      <style>
        body { font-family: Arial; text-align: center; margin-top: 50px; }
        h1 { font-size: 2em; }
        .value { font-size: 1.5em; color: blue; }
        textarea {
          width: 60%;
          height: 100px;
          font-size: 1em;
          margin-top: 20px;
          border-radius: 8px;
          padding: 10px;
        }
      </style>
    </head>
    <body>
      <h1>ESP32 Control Panel</h1>
      <p>want_speed = <span id="speed_val" class="value">0</span></p>
      <p>turn = <span id="turn_val" class="value">0</span></p>
      <textarea id="liveBox" readonly></textarea>

      <script>
        let upHeld = false;
        let downHeld = false;

        document.addEventListener('keydown', function(event) {
          let key = event.key.toLowerCase();
          if (key === 'w') fetch('/adjust?ws=inc');
          else if (key === 's') fetch('/adjust?ws=dec');
          else if (key === 'x') fetch('/set?turn=0');
          else if (key === 'a') fetch('/set?turn=10');
          else if (key === 'd') fetch('/set?turn=-10');
          else if (event.key === 'ArrowUp') upHeld = true;
          else if (event.key === 'ArrowDown') downHeld = true;
        });

        document.addEventListener('keyup', function(event) {
          if (event.key === 'ArrowUp') upHeld = false;
          if (event.key === 'ArrowDown') downHeld = false;
        });

        document.addEventListener('keyup', function(event) {
          if (event.key === 'ArrowUp') {
            upHeld = false;
            fetch('/adjust?want_speed=reset'); // Reset on release
          }
        });

        setInterval(() => {
          if (upHeld) fetch('/holdadjust?dir=up');
          if (downHeld) fetch('/holdadjust?dir=down');
        }, 100); // Poll every 100 ms

        async function fetchValues() {
          const res = await fetch('/values');
          const data = await res.json();
          document.getElementById('speed_val').innerText = data.want_speed;
          document.getElementById('turn_val').innerText = data.turn;
          document.getElementById('liveBox').value = `c: ${data.c}\nd: ${data.d}\ne: ${data.e}`;
        }

        setInterval(fetchValues, 500);
      </script>

    </body>
    </html>
  )rawliteral";

  server.send(200, "text/html", html);
}

void handleSet() {
  if (server.hasArg("want_speed")) want_speed = server.arg("want_speed").toInt();
  if (server.hasArg("turn")) turn = server.arg("turn").toInt();
  server.send(200, "text/plain", "Values updated");
}

void handleValues() {
  String json = "{\"want_speed\":" + String(want_speed) +
                ",\"turn\":" + String(turn) +
                ",\"c\":" + String(c) +
                ",\"d\":" + String(d) +
                ",\"e\":" + String(e) + "}";
  server.send(200, "application/json", json);
}

void handleAdjust() {
  if (server.hasArg("want_speed")) {
    String val = server.arg("want_speed");
    if (val == "inc") {
      want_speed += 0.01;
    } else if (val == "dec") {
      want_speed -= 0.01;
    } else if (val == "reset") {
      want_speed = 0.14;
    }
  }
  server.send(200, "text/plain", "want_speed adjusted");
}

float n=0;
float m=0;
void handleHoldAdjust() {
  static unsigned long lastActionTime = 0;
  static bool blocked = false;

  if (!server.hasArg("dir")) {
    server.send(200, "text/plain", "Missing direction");
    return;
  }

  String dir = server.arg("dir");
  unsigned long now = millis();

  if (blocked && now - lastActionTime < 1000) {
    server.send(200, "text/plain", "Blocked");
    return;
  }

  if (dir == "up") {
    if (n == 0) {
      want_speed += 0.02;
      n = 1;
    }
    if (step1.getSpeedRad() > 1.5 && n == 1) {
      want_speed -= 0.02;
      lastActionTime = now;
      blocked = true;
      n = 0;
    }
  } else if (dir == "down") {
    if (m == 0) {
      want_speed -= 0.02;
      m = 1;
    }
    if (step1.getSpeedRad() < -1.5 && m == 1) {
      want_speed += 0.02;
      lastActionTime = now;
      blocked = true;
      m = 0;
    }
  }

  // Unblock when time passes
  if (now - lastActionTime >= 300) blocked = false;

  server.send(200, "text/plain", "Hold adjust processed");
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

  WiFi.begin(ssid, password);
  Serial.print("Connecting");

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nConnected to WiFi.");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());

  server.on("/", handleRoot);
  //server.on("/set", handleSet);
  server.on("/values", handleValues);
  server.on("/adjust", handleAdjust);
  server.on("/holdadjust", handleHoldAdjust);
  server.begin();
  Serial.println("HTTP server started.");

  mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
  mpu.setGyroRange(MPU6050_RANGE_250_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_44_HZ);

  //Attach motor update ISR to timer to run every STEPPER_INTERVAL_US Î¼s
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



void loop()
{
  server.handleClient();

  // Dummy update of c, d, e every 1 second
  if (millis() - lastUpdate > 1000) {
    lastUpdate = millis();
    c = step1.getSpeedRad(); d=want_speed; e=tilt_x;
  }
  //Static variables are initialised once and then the value is remembered betweeen subsequent calls to this function
  static unsigned long printTimer = 0;  //time of the next print
  static unsigned long loopTimer = 0;
  static unsigned long loopTimer2 = 0;   //time of the next control update
  float dt2 = 0;
  float tiltx = 0.0;             //current tilt angle
  
  
// outer loops
  if (millis() > loopTimer2) {
    loopTimer2 += 100;
    now2 = millis();
    dt2 = (now2 - last_time2)/1000;
    last_time2 = now2;

    outeraddon = pid_2(want_speed-(step1.getSpeedRad()),dt2);
    outeraddon = constrain(outeraddon, -0.03, 0.03);
    outeraddon = outeraddon -0.03;
  }

  //Run the control loop every LOOP_INTERVAL ms inner loop
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



    error =   0.15 - (tilt_x);
    accel = pid_1(error);

    motorspeed += accel * dt;

// Clamp it if needed
    if(motorspeed > 8){
      motorspeed = 8;
    }
    else if(motorspeed < -8){
      motorspeed = -8;
    }


    float accel1 = accel;
    float accel2 = accel;


    step1.setTargetSpeedRad(motorspeed);
    step2.setTargetSpeedRad(-motorspeed);

    step1.setAccelerationRad(accel1+turn);
    step2.setAccelerationRad(accel2-turn);
    k=k+1;





  }



  //Print updates every PRINT_INTERVAL ms

    

  

  
}