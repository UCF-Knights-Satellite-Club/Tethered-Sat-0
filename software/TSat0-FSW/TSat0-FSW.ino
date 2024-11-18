#include <Wire.h>
#include <Adafruit_MMA8451.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP3XX.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Arducam_Mega.h>
#include <Arducam/Platform.h>
#include <SPI.h>
#include <FS.h>
#include <SD.h>
#include <ESP32Servo.h>

// Pin constants
#define CAM_CS 17
#define SCK 14
#define MISO 12
#define MOSI 13
#define SD_CS 15
#define LED_BUILTIN 2
#define SERVO_PIN 4

// Constants
#define SEALEVELPRESSURE_HPA (1013.25)
#define PIC_BUFFER_SIZE 0xff
#define CALIBRATION_COUNT 20            // how many measurements to average when determining initial altitude
#define CONSISTENT_READING_THRESHOLD 5  // how many measurements in a row need to agree to change state
#define ASCENT_RATE_THRESHOLD 0.1//0.2       // altitude change per check to switch from PREFLIGHT to ASCENT
#define MAX_PREFLIGHT_ALTITUDE 3//20       // altitude where PREFLIGHT automatically switches to ASCENT
#define ALTITUDE_CHECK_DELAY 50         // ms between altitude checks
#define FREEFALL_RATE_THRESHOLD -0.01 //-0.5    // altitude change per check to switch from ASCENT to FREEFALL
#define PARACHUTE_DEPLOY_ALTITUDE 6 //80    // altutude to switch from FREEFALL to LANDING
#define ALTITUDE_CHANGE_FILTER_GAIN 0 // between 0 and 1, higher number means each measurement has lower impact on estimate
#define SERVO_STOW_POS 180
#define SERVO_DEPLOY_POS 120


// Define TEST_MODE to enable test mode
//#define TEST_MODE

// Peripheral globals
Adafruit_MMA8451 mma = Adafruit_MMA8451();
Adafruit_BMP3XX bmp;
Adafruit_SSD1306 display(128, 64, &Wire, -1);  //select reset pin (just set to any unused pin)
Arducam_Mega cam(CAM_CS);
Servo servo;

// Function definitions
void write_pic(Arducam_Mega &cam, File dest);

// Globals
static TaskHandle_t check_altitude = NULL;
int pic_num = 0;
char base_dir[20];

typedef enum {
  CALIBRATION,  // establishing altitude baseline
  PREFLIGHT,    // waiting for launch
  ASCENT,       // rising on balloon
  FREEFALL,     // detached from balloon
  LANDING,      // parachute deployed
} FlightState;

FlightState flight_state = CALIBRATION;
int checkCount = 0;

void setup() {
  // LED on for entire setup process
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);

  Serial.begin(115200);

  

#ifdef TEST_MODE
  // setup OLED
  display.begin(SSD1306_SWITCHCAPVCC, 0x3c);  // i2c address
  display.setTextSize(2);
  display.setTextColor(WHITE);
#endif
  
  servo.attach(SERVO_PIN);
  servo.write(SERVO_STOW_POS);

  // setup CAM
  cam.begin();

  //setup SD
  /*if (!SD.begin(CS)) {
        Serial.println(F("Error"));
        Serial.println(F("Card mount failed"));
        while(1);
    }
    uint8_t cardType = SD.cardType();
    if(cardType == CARD_NONE){
        Serial.println(F("Error"));
        Serial.println(F("No SD card attached"));
        while (1);
    }
	
	//check for existence of the file with the given name
	while (SD.exists(fileName)){ 
		fileIterator++;
		fileName = "/EC-" + String(fileIterator) + ".txt";
	}
	
	*/
	
    // Lower SPI speed for stability
    SPI.setClockDivider(SPI_CLOCK_DIV2);

    // Delay to stop first image from being green
    arducamDelayMs(500);
    
    // setup MMA
    if (! mma.begin()) {
        Serial.println("Couldnt start");
        while (1);
    }
    Serial.println("MMA8451 found!");

    // configure MMA
    mma.setRange(MMA8451_RANGE_2_G);
    Serial.print("Range = "); Serial.print(2 << mma.getRange());  
    Serial.println("G");
    
  // setup BMP
  if (!bmp.begin_I2C()) {  // hardware I2C mode, can pass in address & alt Wire
    Serial.println("Could not find a valid BMP3 sensor, check wiring!");
    while (1)
      ;
  }
  // configure BMP
  // Set up oversampling and filter initialization
  bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
  bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);
  bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
  bmp.setOutputDataRate(BMP3_ODR_50_HZ);

  // configure altitude relative to ground (aka. height on power-on)
  //BMP_altitude_setup();

  bmp.performReading();

  // pick base dir
  int i = 0;
  do {
    sprintf(base_dir, "/tsatlog%d", i);
    i++;
  } while (SD.exists(base_dir));
  SD.mkdir(base_dir);
  Serial.print("Data from this run stored in ");
  Serial.println(base_dir);

  xTaskCreatePinnedToCore(
    checkAltitude,     // Function to call
    "Check Altitude",  // Task name
    2048,              // Memory allocated
    NULL,              // Parameters to pass to the function
    1,                 // Priority (higher number = higher priority)
    &check_altitude,   // Task handle
    0                  // Core
  );

  // LED off once setup complete
  digitalWrite(LED_BUILTIN, LOW);
}

void loop() {}

// A parameter of type (void*) is needed to prevent error when creating the task
void checkAltitude(void* parameter) {
  float start_altitude = 0;
  int calibration_count = 0;
  int consistent_reading_count = 0;
  float prev_altitude_change_estimate = 0;
  float altitude_change_estimate = 0;


  // While loop needed to run forever
  while (1) {
    // Check the altitude and do whatever
    //Serial.println("CHECKING THE ALTITUDE");
    float prev_altitude = bmp.readAltitude(SEALEVELPRESSURE_HPA);
    bmp.performReading();
    float absolute_altitude = bmp.readAltitude(SEALEVELPRESSURE_HPA);
    float altitude_change = absolute_altitude - prev_altitude;

    /* get mma data
    mma.read();
    Serial.print("X:\t"); Serial.print(mma.x); 
    Serial.print("\tY:\t"); Serial.print(mma.y); 
    Serial.print("\tZ:\t"); Serial.print(mma.z); 
    */
    

    altitude_change_estimate = (ALTITUDE_CHANGE_FILTER_GAIN * prev_altitude_change_estimate) + (1-ALTITUDE_CHANGE_FILTER_GAIN) * altitude_change;
    prev_altitude_change_estimate = altitude_change_estimate;
    float altitude = absolute_altitude - start_altitude;

    switch (flight_state) {
      case CALIBRATION:
        // average several readings for baseline then move to PRELAUNCH
        start_altitude += absolute_altitude;
        calibration_count++;
        
        Serial.print("Calibrating altitude ");
        Serial.print(calibration_count);
        Serial.print("/");
        Serial.println(CALIBRATION_COUNT);
        
        if (calibration_count >= CALIBRATION_COUNT) {
          start_altitude /= CALIBRATION_COUNT;
          prev_altitude_change_estimate = 0;
          altitude_change_estimate = 0;
          flight_state = PREFLIGHT;
        
          Serial.print("Starting altitude: ");
          Serial.println(start_altitude);
          Serial.println("Calibration complete, moving to PREFLIGHT");
          
        }
        break;
      case PREFLIGHT:
        // move to ASCENT if altitude is rising fast enough or altitude above threshold
        
        if (altitude > MAX_PREFLIGHT_ALTITUDE) {
          flight_state = ASCENT;
          consistent_reading_count = 0;
          
          Serial.println("Max preflight altitude exceeded, moving to ASCENT");
         
        } else if (altitude_change_estimate > ASCENT_RATE_THRESHOLD) {
          consistent_reading_count++;
          if (consistent_reading_count >= CONSISTENT_READING_THRESHOLD) {
            flight_state = ASCENT;
            consistent_reading_count = 0;
            
            Serial.println("Ascent speed threshold exceeded, moving to ASCENT");
            
          }
        } else {
          consistent_reading_count = 0;
        }
        break;
      case ASCENT:
        // move to FREEFALL if altitude drops fast enough
        Serial.println(altitude_change_estimate);
        if (altitude_change_estimate <= FREEFALL_RATE_THRESHOLD) {
          consistent_reading_count++;
          Serial.print("Freefall threshold met: ");
          Serial.println(consistent_reading_count);
          if (consistent_reading_count >= CONSISTENT_READING_THRESHOLD) {
            flight_state = FREEFALL;
        
            digitalWrite(LED_BUILTIN, HIGH);

            consistent_reading_count = 0;
           
            Serial.println("Descent speed threshold exceeded, moving to FREEFALL");
           
          }
        } else {
          consistent_reading_count = 0;
        }
        break;
      case FREEFALL:
        // move to LANDING if below deployment altitude
        if (altitude < PARACHUTE_DEPLOY_ALTITUDE) {
          flight_state = LANDING;
      
          Serial.println("Parachute deployment altitude reached, moving to LANDING");
          servo.write(SERVO_DEPLOY_POS);
          
          
        }
        break;
      case LANDING:
        // deploy parachute, no more state changes
        break;
    }

#ifdef TEST_MODE
    // display code:
    display.clearDisplay();
    display.drawRoundRect(0, 0, 128, 64, 8, WHITE);
    display.setRotation(2);
    display.setCursor(15, 3);
    if (flight_state != CALIBRATION) {
      display.setCursor(altitude>=0?22:10, 8);
      display.print(altitude);
    } else {
      display.setCursor(absolute_altitude>=0?22:10, 8);
      display.print(absolute_altitude);
    }
    display.print(" m");
    display.setCursor(altitude_change_estimate>=0?22:10, 28);
    display.print(altitude_change_estimate * 1000 / ALTITUDE_CHECK_DELAY);
    display.print( " m/s");
    
    display.setCursor(10,48);
    switch (flight_state) {
      case CALIBRATION:
        display.print("CALIBRATE");
        break;
      case PREFLIGHT:
        display.print("PREFLIGHT");
        break;
      case ASCENT:
        display.print("ASCENT");
        break;
      case FREEFALL:
        display.print("FREEFALL");
        break;
      case LANDING:
        display.print("LANDING");
        break;
    }
    display.display();
#endif

    // Delay this task for 50 ms (20 Hz)
    vTaskDelay(ALTITUDE_CHECK_DELAY / portTICK_PERIOD_MS);
  }
}

void write_pic(Arducam_Mega &cam, File dest) {

  uint8_t count = 1;
  uint8_t prev_byte = 0;
  uint8_t cur_byte = 0;
  uint8_t head_flag = 0;
  unsigned int i = 0;
  uint8_t image_buf[PIC_BUFFER_SIZE] = {0};

  while (cam.getReceivedLength())
  {
    // Store current and previous byte
    prev_byte = cur_byte;
    cur_byte = cam.readByte();

    // Write data to buffer
    if (head_flag == 1)
    {
      image_buf[i++]=cur_byte;
      // When buffer is full, write to file
      if (i >= PIC_BUFFER_SIZE)
      {
        dest.write(image_buf, i);
        i = 0;
      }
    }
    // Initialize file on JPEG file start (0xFFD8)
    if (prev_byte == 0xff && cur_byte == 0xd8)
    {
      head_flag = 1;
      //sprintf(name,"/%d.jpg", count);
      count++;
      //Serial.print(F("Saving image..."));
      image_buf[i++]=prev_byte;
      image_buf[i++]=cur_byte;
    }
    // Close file on JPEG file ending (0xFFD9)
    if (prev_byte == 0xff && cur_byte == 0xd9)
    {
      //headFlag = 0;
      dest.write(image_buf, i);
      //i = 0;
      dest.close();
      Serial.println(F("Done"));
      break;
    }
  }
}
