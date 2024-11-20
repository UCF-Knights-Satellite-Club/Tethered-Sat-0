![tsat](https://github.com/user-attachments/assets/e8f95242-9706-4da0-8f7c-fe866dcd503a)
# Tethered-Sat-0
Tethered-Sat-0 or T-Sat-0 is Knights Satellite Club's first CubeSat mission. This a suborbital, tethered balloon mission.

## Hardware
This mission uses one PCB which has been designed in Autodesk EAGLE. The PCB holds several breakout boards, connectors, and an ESP32 devkit for ease of design and testing. The PCB fits into a 1U frame along with an Arducam Mega, a battery, and a parachute deployment system. The burn-wire system consists of two RFM radio modules, one flies with the CubeSat and recieves the signal from the ground station which sends the burn command. Once the burn command is recieved, a Seeduino XIAO will activate a transistor connected to a nichrome wire which melts through a monofilament fishing line and drops T-Sat-0.

## Software
This project utilizes the Arduino ecosystem on an ESP32 and two Seeduino XIAOs. Unless stated otherwise, all libraries can be found in the Arduino IDE Library Manager.

### Prerequisites
- **Arduino ESP32 Core** ([installation instructions](https://docs.espressif.com/projects/arduino-esp32/en/latest/installing.html))
- **Seeduino Arduino IDE Support** ([installation instructions](https://wiki.seeedstudio.com/Seeed_Arduino_Boards/))
- **Radiohead** by Mike McCauley (Arduino Library Manager version is old, [download directly](http://www.airspayce.com/mikem/arduino/RadioHead/))
- **Adafruit MMA8451 Library** by Adafruit
- **Adafruit BMP3XX Library** by Adafruit
- **Adafruit GFX Library** by Adafruit
- **Adafruit SSD1306** by Adafruit
- **Adrucam_Mega** by Arducam
- **ESP32Servo** by Kevin Harrington
