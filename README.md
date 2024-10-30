ESP32CAM+V7RC FPV視訊小車

![two rc caar image](/media/two_cars.jpg)

# What is this code
This code turns ESP32cam into a RC car system with FPV camera, and able to control two motors, one servo, two LEDs  at the same time.
The system design is built around V7RC mobile app, a RC controller app that can run on [android](https://play.google.com/store/apps/details?id=com.v7idea.v7rcliteandroidsdkversion) and [IOS](https://apps.apple.com/tw/app/v7rc/id1390983964), providing motor control, camera view and even more.

# How to run this code

## Install
This code is written under arduino envirement. So you need to install arduino then add ESP32 support to it. Search online to get help.
Next in library manager, install [WiFiManager from tzpu](https://github.com/tzapu/WiFiManager) and [ESP32Servo from madhephaestus](https://github.com/madhephaestus/ESP32Servo). these are underlying third party libries this code depend on.
This is simply one arduino project not a library. Download two files from src directory and put them into the same arduino project directory. Set the target board type to "**ESP32 Dev Board**" then compile should pass without problem.

## Boot
Upon power on, it will altomatically try to connect to previous connected WiFi. If failed, it will enter WiFi AP mode and ready for configure. Or hold GPIO-0 to GND for more than 3 seconds after boot, it enters configurion state as well.

After a successful boot and connected to external WiFi AP, this system will listen to UDP://IP:6188 for control message. Not limited to V7RC app but anything that can send UDP data can control it.
Separate from UDP control flow, there's another HTTP://IP:81/stream/ for IP camera access. Even a web browser can view it without problem. But only one client can access at the same time.

## Connectivity 
First to note: This works solely on WiFi, no BLE support.
This code is set to work as a WiFi client, an external WiFi AP is needed. Only when setting up WiFi credential, it switchs to WiFi Ap mode and configure it by connecting to it using cellphone.
When the device is in configuration state, a new AP named "ESP32-XXXXXX" can be found if you do a wifi search on your phone. Connect to it and open "http://192.168.4.1" on your browser app.
This is where to set SSID, password, and even static IP address. After saving data, it will reboot and try to connect to that WiFi station.

## Connect to V7RC app
### App settings
* Set the connection method to "WIFI", not "Bluetooth".
* You might need to first check the DHCP setting on the router or force to set a specific device IP for the ESP32CAM device. Then enter the right IP address for control device and IPCamera.
For example, if the device IP is 192.168.1.234, then the settings will be "192.168.1.234" for "IP" and "6188" for Port. And the IPCamera will be "http://192.168.1.234:81/stream".
* Set the interface to "car" or "tank", and change the coresponding channels by pressing "CH" button on the top left.
* Now try to move the joystick and see if everything works.

### Supported V7RC Message Protocols
#### Motor control message
This system supports basic two types: SRV and SRT.
The message will be like: **SRV1111222233334444#** or **SRT1111222233334444#** where 1, 2, 3, and 4 means four different motor channels. These values are generic PWM control signal ranging from 1000 to 2000, and 1500 normally means its center/netual position.
Channel 1 is for servo control, channel 2 and channel 3 are for motor control.
#### LED control message
This system supports LED message, not LE2 message.
The message will be like: **LED1111222233334444#** where 1, 2, 3, and 4 means four different LEDs. Each LED comes with four color settings: Red, Green, Blue and Motion. Here in this code we only take first two LED channels and consider the Red setting of it.
Open the "extra" menu on the top right connor, L1 means LED1 and L2 means LED2. When light up, the coresponded GPIO will set to HIGH. Other six LED channels are not used in here.
Note that unlike motor message sends periodcally, LED message only sends when button pressed. Sometime the light status might go wrong after boot. Just reset them by press any of the first four led button to resend LED message.

# Hardware requirement
![Pin to pin connection](/media/wiring.png)
## Pin Connection
* Require 5V power source to run everything.
* The servo is connected to GPIO-2 directly.
* Uses GPIO-12 and GPIO-13 to control one motor, and GPIO-14 and GPIO-15 to control the other. **Do not connect motors directly** to ESP32CAM, use motor driver IC to do so. 
* Two LED channels are on GPIO-4 and GPIO-16.
* GPIO-0 for entering configuration portal.

## External devices
### Camera
This code is specificly build for ESP32CAM with OV2640 camera. It is quite easy to fulfill since almost all ESP32CAM seller has this combination.

### WiFi on the fly configuration
A button on GPIO-0 and GND is important, it can force this system go into WiFi AP mode and then you can set WiFi credentials on the fly. Without it the system only goes into AP mode when previous AP is not found.

### Power source
The main power rail is 5V, which is because many experienced unpredictable crash/reset when running with 3.3V. Also this system is targeted for a toy servo like SG90 which designed to work on 5V, and simple DC motors that runs faster and smoother at 5V. So if you are using single Li-ION battery, you need a 5V boost module.

### Servo and motors
The servo is controled directly from ESP32CAM using hardware timer. Standard 1000 to 2000 us signal within 2ms signal is applied, use a simple SG90 servo to check. 
Two motors is driven by external motor driver IC. Remember to add a tiny capacitor on each motor to absorb voltage spike. Note that we can only squeeze two pins for each motor, that means you can use L9110s or DRV8833 but not TB6612fng.

### LED
Two LED channels are driven directly from GPIO so the voltage is 3.3V not 5V and current is limited.
For GPIO-4, it is connected to onboard flash LED. It's too bright normally you don't want it to light up there so just get rid of it and you can gain full control on GPIO-4.
For GPIO-16, set arduino target board from "AiThinker ESP32-CAM" to "**ESP32 Dev Board**" and check the "**PSRAM**" is set to "**Disabled**". If you don't do this and still use GPIO-16, system will crash immediately after boot.
# Troubleshooting
## Motor direction reversed
As DC motors can run both direction, this happens very often. Just swap two connected cables.
## Turning mechanism not functional
There are two comon types of RC car turning mechanism: using a servo that can fine-tuning angle or a motor with gears and spring that only choose left or right. Unfortunally, most of the cheap RC car choose the second one.
For that, the motor needs a burst power to turn the gear and hold at nearly full power. PWM mode won't provide this power to do so. Set MOTORx_USE_PWM to false to set the behavior to full power only.
And also the tank style which uses two sets of motors on different side of the car. For this, simply leave both motor output to PWM mode.
## WiFi signal unstable
![add 1pf between antenna end and GND](/media/antenna_hack.jpg)
ESP32CAM built-in PCB antenna sometimes cause connection problems. There's a dirty hack: add 1pF capacitor at the end of the antenna and GND. Note the outer chassis is connected to GND as well, no need to scrap the solder mask.
If that does not work, an overall solution is using an external antenna instead.
