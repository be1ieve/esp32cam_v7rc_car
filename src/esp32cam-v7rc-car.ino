
/*
 * This program needs an external wifi network to connect to.
 * Using WiFiManager, there's no need to write wifi credencial here.
 * When booted, onboard LED will turn on for a few second. Hold IO-0 to GND for three seconds.
 * Then search for wifi on your cellphone, a new ESP32-XXXXXX AP will show up.
 * Connect to this AP and go to http://192.168.4.1/ to update your wifi settings.
 * 
 * V7RC control send to UDP://<IP>:6188
 * Channel-1 controls the servo on GPIO-2
 * Channel-2 controls one motor on GPIO-12 and GPIO-13
 * Channel-3 controls the other motor on GPIO-14 and GPIO-15
 * 
 * Webcam streaming on http://<IP>:81/stream
 * Camera is striped down to ai-thinker esp32cam only. Find coresponding settings from official CameraWebServer example.
 * Default resolution is set to 320x240. To change it, check official example and change it in cameraInit() inside camera.h.
 * 
 * GPIO-4 is free but still connects to onboard flash LED
 * GPIO-16 can be free if PSRAM is not used
 * 
 * WiFiManager and ESP32Servo libraries come from github.
 * Many thanks to tzapu and madhephaestus!
 * Get required libries here:
 * https://github.com/tzapu/WiFiManager
 * https://github.com/madhephaestus/ESP32Servo
 */

#include <WiFi.h>
#include <WiFiManager.h>
#include <WiFiUdp.h>
#include "esp_timer.h"

#include <ESP32PWM.h>
#include <ESP32Servo.h>

#include "camera.h" // not a library, just put those copied things aside.

#define WM_TRIGGER_PIN 0 // wifimanager trigger pin
#define NOTIFY_LED 33 // Use esp32cam onboard red led, inverted

/*
 * Below are servo configurations. Change these to fit this device.
 * Since every servo needs to centered before install, still some fine tuning is required.
 * For turning servo, lower value means left and higher value means right.
 */
#define TURNING_SERVO_PIN 2

/*
 * For L9110s and DRV8833, only two pins required.
 * But they only activate when two pins are in different state.
 * That means the PWM value corespond to the output speed is reversed:
 * When direction pin is HIGH, higher PWM value means slower speed, 
 * and when direction pin is LOW, higher PWM value means faster speed.
 * 
 * Default we should set direction to LOW and speed to 0
 */
#define MOTOR1_PWM_PIN 12
#define MOTOR1_DIR_PIN 13
#define MOTOR2_PWM_PIN 15
#define MOTOR2_DIR_PIN 14

/*
 * variables for servo control
 */
#define SERVO_DEFAULT_ANGLE 90
int turningServoCurrentPosition = SERVO_DEFAULT_ANGLE; // default position at 90 degrees.
int turningServoTargetPosition = SERVO_DEFAULT_ANGLE; // tracking target position in loop()
Servo turningServo;

/*
 * variables for motor control
 */
ESP32PWM pwm1;
ESP32PWM pwm2;
boolean pwm1CurrentDirection = false;
boolean pwm1TargetDirection = false;
int pwm1CurrentSpeed = 0;
int pwm1TargetSpeed = 0;
boolean pwm2CurrentDirection = false;
boolean pwm2TargetDirection = false;
int pwm2CurrentSpeed = 0;
int pwm2TargetSpeed = 0;

/*
 * variables for V7RC remote control on UDP
 */
WiFiUDP udp;
const int udpPort = 6188; // For V7RC control, default 6188
bool udp_packet_not_empty = false;
bool udp_packet_loaded = false;
char packetBuffer[64];

/*
 * WiFi Manager for selecting new wifi and reconnecting to existing wifi
 */
WiFiManager wm;

/*
 * Failsafe settings to force device to stop
 */
#define FAILSAFE_MAX 500 // max steps without updating data
int failsafeCounter = FAILSAFE_MAX;



void setup(){
  Serial.begin(115200);      // initialize serial communication
  WiFi.mode(WIFI_STA); // explicitly set mode, esp defaults to STA+AP  

  pinMode(WM_TRIGGER_PIN, INPUT_PULLUP);
  pinMode(NOTIFY_LED, OUTPUT); // onboard led, inverted

  digitalWrite(NOTIFY_LED, LOW); // LED ON
  delay(3000); // wait for 3 seconds
  if (digitalRead(WM_TRIGGER_PIN) == LOW){ // pressed
    wm.setShowStaticFields(true); // enable to set static IP
    Serial.println("Starting config portal...");
    wm.startConfigPortal();
  }
  else{
    Serial.println("Connect using existing wifi setting");
    wm.autoConnect();
  }
  // check if wifi is connected. iIf not, just reboot
  if(!WiFi.status() == WL_CONNECTED){
    Serial.println("failed to connect and hit timeout");
    delay(2000);
    ESP.restart();
    delay(3000); // block everything to make sure it reboots
  }
  digitalWrite(NOTIFY_LED, HIGH); // LED OFF when connected
  
  ESP32PWM::allocateTimer(1); // Timer 0 used by camera, use timer1 instead
  turningServo.setPeriodHertz(50); // standard 50 hz servo
  turningServo.attach(TURNING_SERVO_PIN, 1000, 2000);
  turningServo.write(turningServoCurrentPosition);
  
  ESP32PWM::allocateTimer(2); // use timer2 for motors
  pinMode(MOTOR1_DIR_PIN, OUTPUT);
  pinMode(MOTOR1_PWM_PIN, OUTPUT);
  pwm1.attachPin(MOTOR1_PWM_PIN,500,8);
  //Serial.printf("PWM1 on timer %d\n", pwm1.getTimer());

  pinMode(MOTOR2_DIR_PIN, OUTPUT);
  pinMode(MOTOR2_PWM_PIN, OUTPUT);
  pwm2.attachPin(MOTOR2_PWM_PIN,500,8);
  //Serial.printf("PWM1 on timer %d\n", pwm1.getTimer());

  cameraInit();

  udp.begin(udpPort);
  startCameraServer();

  Serial.print("Camera on 'http://");
  Serial.print(WiFi.localIP());
  Serial.print(":81/stream");
  Serial.println("' to connect");

  Serial.print("V7RC control port on: UDP://");
  Serial.print(WiFi.localIP());
  Serial.printf(":%d\n",udpPort);
}

/*
 * Decode basic two V7RC format: SRV1000200015001500# and SRT1000200015001500#
 * Both contains 4 channels ranging from 1000 to 2000
 * Assume the remote controller is set to default: outputs between 1000 to 2000 and centered at 1500.
*/
void loop(){
  if(udp.parsePacket() > 0){ // UDP packet arrived
    udp.read(packetBuffer, 60); // read a packet with buffer larger enough to fit more than one command
    String receiveData = String(packetBuffer);
    receiveData.trim(); // Remove leading and trailing whitespaces in place
    if (receiveData.startsWith("SRT") || receiveData.startsWith("SRV")){ // V7RC packet format see comments above
      int ch1_data=(receiveData.substring(3,7)).toInt(); 
      int ch2_data=(receiveData.substring(7,11)).toInt();
      int ch3_data=(receiveData.substring(11,15)).toInt(); 
      int ch4_data=(receiveData.substring(15,19)).toInt();
      Serial.printf("CH1: %d, CH2: %d, CH3: %d, CH4: %d\n", ch1_data, ch2_data, ch3_data, ch4_data);
      
      failsafeCounter = FAILSAFE_MAX; // reset failsafe value when valid data received

      turningServoTargetPosition = map(ch1_data, 1000, 2000, 0, 180); // mapping channel value to angle
      
      if(ch2_data > 1550){ // forward
        pwm1TargetDirection = false;
        pwm1TargetSpeed = map(ch2_data, 1500, 2000, 0, 255);
      } else if(ch2_data < 1450){ // backward
        pwm1TargetDirection = true;
        pwm1TargetSpeed = map(ch2_data, 1500, 1000, 0, 255);
      } else{ // stop
        //pwm1TargetDirection = false;
        //if(pwm1TargetDirection) pwm1TargetSpeed = 255;
        //else pwm1TargetSpeed = 0;
        pwm1TargetSpeed = 0;
      }

      if(ch3_data > 1550){ // forward
        pwm2TargetDirection = false;
        pwm2TargetSpeed = map(ch3_data, 1500, 2000, 0, 255);
      } else if(ch3_data < 1450){ // backward
        pwm2TargetDirection = true;
        pwm2TargetSpeed = map(ch3_data, 1500, 1000, 0, 255);
      } else{ // stop
        //pwm2TargetDirection = false;
        //if(pwm2TargetDirection) pwm2TargetSpeed = 255;
        //else pwm2TargetSpeed = 0;
        pwm2TargetSpeed = 0;
      }
    }
    else{ // not a v7rc packet
      Serial.println(packetBuffer);
    }
  } // end of parsing packet

  /*
   * Failsave enables when no valid new UDP data in time. 
   * It overwrites target values to let servo and motors going back to natural position.
   */
  if(failsafeCounter <= 0){
    turningServoTargetPosition = SERVO_DEFAULT_ANGLE;
    pwm1TargetDirection = false;
    pwm1TargetSpeed = 0;
    pwm2TargetDirection = false;
    pwm2TargetSpeed = 0;
    failsafeCounter = FAILSAFE_MAX / 2; // reset counter with a lower number
  }
  else failsafeCounter--;

  /*
   * Finally slowly turn servo and motors to target position. 
   * Changing speed too fast will crash esp32
   */
  if (turningServoCurrentPosition < turningServoTargetPosition) turningServoCurrentPosition++;
  else if (turningServoCurrentPosition > turningServoTargetPosition) turningServoCurrentPosition--;
  turningServo.write(turningServoCurrentPosition);

  if(pwm1CurrentDirection != pwm1TargetDirection){ // different direction
    if(pwm1CurrentSpeed > 0) pwm1CurrentSpeed--; // speed down
    else { // change direction
      pwm1CurrentDirection = pwm1TargetDirection;
      digitalWrite(MOTOR1_DIR_PIN, pwm1CurrentDirection);
    }
  } else{ // same direction
    if(pwm1CurrentSpeed > pwm1TargetSpeed) pwm1CurrentSpeed--;
    else pwm1CurrentSpeed++;
  }
  //pwm1.write(pwm1CurrentSpeed);
  if(pwm1CurrentDirection) pwm1.write(255-pwm1CurrentSpeed);
  else pwm1.write(pwm1CurrentSpeed);

  if(pwm2CurrentDirection != pwm2TargetDirection){ // different direction
    if(pwm2CurrentSpeed > 0) pwm2CurrentSpeed--; // speed down
    else { // change direction
      pwm2CurrentDirection = pwm2TargetDirection;
      digitalWrite(MOTOR2_DIR_PIN, pwm2CurrentDirection);
    }
  } else{ // same direction
    if(pwm2CurrentSpeed > pwm2TargetSpeed) pwm2CurrentSpeed--;
    else pwm2CurrentSpeed++;
  }
  //pwm2.write(pwm2CurrentSpeed);
  if(pwm2CurrentDirection) pwm2.write(255-pwm2CurrentSpeed);
  else pwm2.write(pwm2CurrentSpeed);

  delay(2); // one step delay
}
