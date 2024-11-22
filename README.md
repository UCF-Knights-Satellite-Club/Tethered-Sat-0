![tsat](https://github.com/user-attachments/assets/e8f95242-9706-4da0-8f7c-fe866dcd503a)
# Tethered-Sat-0
Tethered-Sat-0 or T-Sat-0 is Knights Satellite Club's first CubeSat mission. This a suborbital, tethered balloon mission.

## Hardware
This mission uses one PCB which has been designed in Autodesk EAGLE. The PCB holds several breakout boards, connectors, and an ESP32 devkit for ease of design and testing. The PCB fits into a 1U frame along with an Arducam Mega, a battery, and a parachute deployment system. The burn-wire system consists of two RFM radio modules, one flies with the CubeSat and recieves the signal from the ground station which sends the burn command. Once the burn command is recieved, a Seeduino XIAO will activate a transistor connected to a nichrome wire which melts through a monofilament fishing line and drops T-Sat-0.

## Software
This project utilizes the Arduino ecosystem on a **DOIT ESP32 DEVKIT V1** and two **Seeeduino XIAO**s. Unless stated otherwise, all libraries can be found in the Arduino IDE Library Manager.

### Prerequisites
- **Arduino ESP32 Core** ([installation instructions](https://docs.espressif.com/projects/arduino-esp32/en/latest/installing.html))
- **Seeeduino Arduino IDE Support** ([installation instructions](https://wiki.seeedstudio.com/Seeed_Arduino_Boards/))
- **Radiohead** by Mike McCauley (Arduino IDE Library Manager version is old, [download directly](http://www.airspayce.com/mikem/arduino/RadioHead/))
- **Adafruit MMA8451 Library** by Adafruit
- **Adafruit BMP3XX Library** by Adafruit
- **Adafruit GFX Library** by Adafruit
- **Adafruit SSD1306** by Adafruit
- **Arducam_Mega** by Arducam
- **ESP32Servo** by Kevin Harrington

### Explanation
The flight software has five states. The first state is `CALIBRATION` which averages several altitude readings to determine a launch altitude. Next is `PREFLIGHT` which runs when T-Sat-0 is on the ground and waiting to be released on the balloon. High acceleration or breaking an altitude threshold will move to the `ASCENT` state. In this state, T-Sat-0 monitors acceleration to determine when it is in free fall. In free fall, the accelerometer will read close to zero. Once this condition is met, the state moves to `FREEFALL` where altitude is compared to the deployment altitude. Once deployment altitude is reached, the parachute is deployed and the state switches to `LANDING`.
