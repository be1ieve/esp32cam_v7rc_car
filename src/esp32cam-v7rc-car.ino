
/*
 * This program needs an external wifi network, use existing one or wifi sharing AP on your phone.
 * 
 * There's no need to write wifi credencial here, WiFiManager kicks in if needed:
 * When booted, it will try to connect to previous connected network. If failed, WiFiManager runs automatically.
 * Another way to force enable WiFiManager is: Hold <WM_TRIGGER_PIN> to GND for three seconds right after power on.
 *
 * When WiFiManager activates, this device will switch to WiFi AP mode.
 * Search for wifi on your phone or computer, a new "ESP32-XXXXXXXX" AP will show up.
 * Connect to this AP and go to http://192.168.4.1/ to update your wifi settings.
 * 
 * V7RC app will send data to UDP://<IP>:6188
 * Channel-1 controls the servo on GPIO-2
 * Channel-2 controls one motor on GPIO-12 and GPIO-13
 * Channel-3 controls the other motor on GPIO-14 and GPIO-15
 * 
 * Webcam is streaming on http://<IP>:81/stream, same URL as official example.
 * Most of the settings are moved into camera.h
 * Camera config is striped down to ai-thinker esp32cam only. Find coresponding settings from official CameraWebServer example.
 * Default resolution is set to 320x240. Don't know why, but 160x120(QQVGA) setting crashs most of the time.
 * To change the camera resolution, it's in cameraInit() inside camera.h.
 * 
 * There are also two LED channels available for ESP32CAM: GPIO-4 and GPIO-16
 * Note that GPIO-4 is free but it connects to onboard super bright flash LED.
 * To use GPIO-4, consider to remove or break onboard LED.
 * GPIO-16 is a little tricky, you need to turn off PSRAM support from compiler settings:
 * 1. Change board type from "AiThinker ESP32-CAM" to "ESP32 Dev module".
 * 2. Check the PSRAM section is set to "Disabled".
 * 3. In camera.h file, make sure "config.fb_location = CAMERA_FB_IN_DRAM". This will also restrict "config.frame_size" to lower resolution.
 * 
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

/*
 * Two LED channel available, but no PWM support, just ON and OFF state.
 * GPIO-4 and GPIO-16 can be used with caution.
 * Set to -1 to disable it.
 * Also, those LED will blink when disconnected from controller.
 */
#define LED1_PIN 4
#define LED2_PIN 16

/*
 * Choose a GPIO pin and connect to a button.
 * This sets internal pull-up resistor on, so just connect the other side to GND is enough.
 */
#define WM_TRIGGER_PIN 0 // wifimanager trigger pin

/*
 * An almost unnoticeable red LED on board.
 * Turn on when boot, turn off when external wifi connected.
 * Useless if LED channel is used because those two will blink when no UDP packet received. 
 */
#define NOTIFY_LED 33 // Use esp32cam onboard red led, GPIO-33. On/Off status is inverted.

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
 * For example if we want to stop the motor, we need to do either: 
 *   Set DIR_PIN to LOW and set PWM_PIN to MAX, or
 *   Set DIR_PIN to HIGH and set PWM_PIN to 0 
 *
 * It doesn't matter which pin connects to which side of the motor. If motor spins backward, just swap it.
 *
 */
#define MOTOR1_PWM_PIN 15
#define MOTOR1_DIR_PIN 14
#define MOTOR2_PWM_PIN 12
#define MOTOR2_DIR_PIN 13

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
char packetBuffer[64];

/*
 * variables for LED control
 */
boolean led1State = false;
boolean led2State = false;


/*
 * WiFi Manager for selecting new wifi and reconnecting to existing wifi
 */
WiFiManager wm;

/*
 * Failsafe settings to force device to stop
 */
#define STEP_DELAY 2
#define FAILSAFE_MAX 1000 // max steps without updating data
int failsafeCounter = FAILSAFE_MAX;
boolean failsafeLedState = true;

void setup(){
  Serial.begin(115200);      // initialize serial communication
  WiFi.mode(WIFI_STA); // explicitly set mode, esp defaults to STA+AP  

  pinMode(WM_TRIGGER_PIN, INPUT_PULLUP);
  pinMode(NOTIFY_LED, OUTPUT); // onboard led, inverted
  digitalWrite(NOTIFY_LED, LOW); // LED ON

  pinMode(LED1_PIN, OUTPUT);
  pinMode(LED2_PIN, OUTPUT);
  digitalWrite(LED1_PIN, LOW); // LED OFF
  digitalWrite(LED2_PIN, LOW); // LED OFF
  

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
  
  Serial.println("wifi init ready");

  ESP32PWM::allocateTimer(1); // Timer 0 used by camera, use timer1 instead
  turningServo.setPeriodHertz(50); // standard 50 hz servo
  turningServo.attach(TURNING_SERVO_PIN, 1000, 2000);
  turningServo.write(turningServoCurrentPosition);
  
  ESP32PWM::allocateTimer(2); // use timer2 for motors
  pinMode(MOTOR1_DIR_PIN, OUTPUT);
  pinMode(MOTOR1_PWM_PIN, OUTPUT);
  pwm1.attachPin(MOTOR1_PWM_PIN,500,8); // 8bit resolution, 0-255
  //Serial.printf("PWM1 on timer %d\n", pwm1.getTimer());

  pinMode(MOTOR2_DIR_PIN, OUTPUT);
  pinMode(MOTOR2_PWM_PIN, OUTPUT);
  pwm2.attachPin(MOTOR2_PWM_PIN,500,8); // 8bit resolution, 0-255
  //Serial.printf("PWM1 on timer %d\n", pwm1.getTimer());

  Serial.println("motor init ready");

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
 * Decode basic V7RC formats:
 * Motor control: SRV1000200015001500# or SRT1000200015001500#
 * Both contains 4 channels ranging from 1000 to 2000
 * Assume the remote controller is set to default: outputs between 1000 to 2000 and centered at 1500.
 *
 * LED control: LED0000FFFA00000000# 
 * Contains 4 full color LED channels ranging from 0000 to FFFA
 * 4 HEX values represents: Red, Green, Blue, and Motion
 * Here we only detect if red value is 0 or not
 * 
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
      } else{ // stop without changing direction
        pwm1TargetSpeed = 0;
      }

      if(ch3_data > 1550){ // forward
        pwm2TargetDirection = false;
        pwm2TargetSpeed = map(ch3_data, 1500, 2000, 0, 255);
      } else if(ch3_data < 1450){ // backward
        pwm2TargetDirection = true;
        pwm2TargetSpeed = map(ch3_data, 1500, 1000, 0, 255);
      } else{ // stop without changing direction
        pwm2TargetSpeed = 0;
      }
    }
    else if(receiveData.startsWith("LED")){
      if(receiveData.charAt(3) == '0') led1State = false;
      else led1State = true;
      if(receiveData.charAt(7) == '0') led2State = false;
      else led2State = true;
    }
    else{ // not a v7rc packet
      Serial.println(packetBuffer);
    }
  } // end of parsing packet

  /*
   * Failsave enables when no valid new UDP data in time. 
   * It overwrites target values to let servo and motors going back to natural position.
   * And then flash both LED to notify user.
   */
  if(failsafeCounter <= 0){
    turningServoTargetPosition = SERVO_DEFAULT_ANGLE;
    pwm1TargetDirection = false;
    pwm1TargetSpeed = 0;
    pwm2TargetDirection = false;
    pwm2TargetSpeed = 0;

    digitalWrite(LED1_PIN, failsafeLedState);
    digitalWrite(LED2_PIN, failsafeLedState);
    failsafeLedState = !failsafeLedState;

    failsafeCounter = FAILSAFE_MAX /2; // reset counter with a lower number
  }
  else if(failsafeCounter > FAILSAFE_MAX /2 ){ // back to normal
    digitalWrite(LED1_PIN, led1State);
    digitalWrite(LED2_PIN, led2State);
  }
  failsafeCounter--;
  
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
    else if(pwm1CurrentSpeed < pwm1TargetSpeed) pwm1CurrentSpeed++;
  }
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
    else if(pwm2CurrentSpeed < pwm2TargetSpeed)pwm2CurrentSpeed++;
  }
  if(pwm2CurrentDirection) pwm2.write(255-pwm2CurrentSpeed);
  else pwm2.write(pwm2CurrentSpeed);

  delay(STEP_DELAY); // one step delay
}
