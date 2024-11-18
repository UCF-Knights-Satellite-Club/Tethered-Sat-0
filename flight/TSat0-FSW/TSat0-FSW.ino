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
#define SCK 14angle of the shaft. On standard servos a parameter value of 1000 is fully counter-clockwise
#define MISO 12
#define MOSI 13
#define SD_CS 15
#define LED_BUILTIN 2
#define SERVO_PWM 4

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

//uint8_t count = 1;
char name[20] = { 0 };
File outFile;
uint8_t imageDataPrev = 0;
uint8_t imageData = 0;
uint8_t headFlag = 0;
unsigned int i = 0;
uint8_t imageBuff[PIC_BUFFER_SIZE] = { 0 };

File myFile;

// Function initializations:
void recordMMA();
void recordBMP();
void BMP_altitude_setup();
void take_image_save();

// Globals



float LOCAL_P = 0;

int FILENUM_CAM = 1;   //camera var
int FILENUM_DATA = 1;  //camera var

String dataString;                                         //string to write to file
int fileIterator = 0;                                      //filename number
String fileName = "/EC-" + String(fileIterator) + ".txt";  //filename

static TaskHandle_t check_altitude = NULL;

typedef enum {
  CALIBRATION,  // establishing altitude baseline
  PREFLIGHT,    // waiting for launch
  ASCENT,       // rising on balloon
  FREEFALL,     // detached from balloon
  LANDING,      // parachute deployed
} FlightState;

FlightState flight_state = CALIBRATION;
int checkCount = 0;

// BMP Globals:
float temperature;
float pressure;
float alt;

float CURRENT_TIME = 0;
float TIME_STATUS;
//float PHOTO_TIME_ELAPSED = 0;

float time1 = 0;
float time2 = 0;
float singleMainLoopTime = 0;
float continuousLoopTime = 0;



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
  
  servo.attach(SERVO_PWM);
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

void loop() {

  




  if (continuousLoopTime > 10000) {

    //take_image_save();
    continuousLoopTime = 0;
  }
  time1 = millis();

  //float alt;
  alt = bmp.readAltitude(LOCAL_P);

/*
#ifdef TEST_MODE
  // display code:
  display.clearDisplay();
  display.drawRoundRect(0, 0, 127, 63, 8, WHITE);
  display.setRotation(2);
  display.setCursor(20, 20);









  //Serial.print("Current altitude = ");
  //Serial.print(alt);
  //Serial.println(" m");

  // more display code:
  display.print(alt);
  display.print(" m");
  display.display();
#endif
*/

  // function call: read temp. save to global
  recordBMP();

  // function call: read gyro. save to global

  // if alt = going_up_alt or greater than: parachute_auth_flag = 1

  // if alt = going_down_alt or less than AND parachute_auth_flag = 1: deploy parachute

  // record data
  //record_data_save();

  time2 = millis();
  singleMainLoopTime = time2 - time1;
  continuousLoopTime += singleMainLoopTime;
}





void BMP_altitude_setup() {
  //double T, P;
  for (int i = 0; i < 4; i++) {
    if (!bmp.performReading()) {
      Serial.println("Failed to perform reading :(");
      return;
    }

    delay(2000);

    LOCAL_P = bmp.pressure / 100.0;  //pressure in hPa

    Serial.print("Local init. P = ");
    Serial.println(LOCAL_P);

    delay(2000);
  }

  Serial.print("FINAL init. P = ");
  Serial.println(LOCAL_P);
}


// can write all data to one single .txt file
void record_data_save() {


  myFile = SD.open(fileName, "w");


  // dataString format: time, temperature, pressure, altitude, parachute_trig_flag,...
  // print to file
  dataString = String(float(millis())) + ", " + String(temperature) + ", " + String(pressure) + ", " + String(alt);  // NOT COMPLETE

  if (myFile) {
    Serial.println("writing to text file successfully");
    myFile.println(dataString);
    myFile.close();  //good
    Serial.println("Done...");
  } else {
    Serial.println("error opening .txt");
  }
}

void take_image_save() {
  Serial.println("START take_image_save");


  digitalWrite(LED_BUILTIN, HIGH);

  // Take picture
  cam.takePicture(CAM_IMAGE_MODE_WQXGA2, CAM_IMAGE_PIX_FMT_JPG);

  // Save image
  while (cam.getReceivedLength()) {
    // Store current and previous byte
    imageDataPrev = imageData;
    imageData = cam.readByte();

    // Write data to buffer
    if (headFlag == 1) {
      imageBuff[i++] = imageData;
      // When buffer is full, write to file
      if (i >= PIC_BUFFER_SIZE) {
        outFile.write(imageBuff, i);
        i = 0;
      }
    }
    // Initialize file on JPEG file start (0xFFD8)
    if (imageDataPrev == 0xff && imageData == 0xd8) {
      headFlag = 1;
      sprintf(name, "/%d.jpg", FILENUM_CAM);

      // ensure filename does not already exist
      while (SD.exists(name)) {
        FILENUM_CAM++;
        sprintf(name, "/%d.jpg", FILENUM_CAM);
      }

      //count++;

      //Serial.print(F("Saving image..."));
      outFile = SD.open(name, "w");
      if (!outFile) {
        Serial.println(F("Error"));
        Serial.println(F("File open failed"));
        while (1)
          ;
      }
      imageBuff[i++] = imageDataPrev;
      imageBuff[i++] = imageData;
    }
    // Close file on JPEG file ending (0xFFD9)
    if (imageDataPrev == 0xff && imageData == 0xd9) {
      headFlag = 0;
      outFile.write(imageBuff, i);
      i = 0;
      outFile.close();
      //Serial.println(F("Done"));
      digitalWrite(LED_BUILTIN, LOW);
      Serial.println("STOP take_image_save");
      break;
    }
  }

  //PHOTO_TIME = millis(); //timestamp of when a photo is taken
}




void recordBMP() {
#ifdef TEST_MODE
  if (!bmp.performReading()) {
    Serial.println("Failed to perform reading :(");
    return;
  }
#endif


  //Serial.print("Temperature = ");
  //Serial.print(bmp.temperature);
  //Serial.println(" *C");

  temperature = bmp.temperature;


  //Serial.print("Pressure = ");
  //Serial.print(bmp.pressure / 100.0);
  //Serial.println(" hPa");

  pressure = bmp.pressure / 100.0;

  //Serial.println();
  //delay(2000);
}

void recordMMA() {

  /*  
    // Read the 'raw' data in 14-bit counts
    mma.read();
    Serial.print("X:\t"); Serial.print(mma.x); 
    Serial.print("\tY:\t"); Serial.print(mma.y); 
    Serial.print("\tZ:\t"); Serial.print(mma.z); 
    Serial.println();

    // Get a new sensor event
    sensors_event_t event; 
    mma.getEvent(&event);

    // Display the results (acceleration is measured in m/s^2)
    Serial.print("X: \t"); Serial.print(event.acceleration.x); Serial.print("\t");
    Serial.print("Y: \t"); Serial.print(event.acceleration.y); Serial.print("\t");
    Serial.print("Z: \t"); Serial.print(event.acceleration.z); Serial.print("\t");
    Serial.println("m/s^2 ");
    */

  // Get the orientation of the sensor
  uint8_t o = mma.getOrientation();

  switch (o) {
    case MMA8451_PL_PUF:
      Serial.println("Portrait Up Front");
      break;
    case MMA8451_PL_PUB:
      Serial.println("Portrait Up Back");
      break;
    case MMA8451_PL_PDF:
      Serial.println("Portrait Down Front");
      break;
    case MMA8451_PL_PDB:
      Serial.println("Portrait Down Back");
      break;
    case MMA8451_PL_LRF:
      Serial.println("Landscape Right Front");
      break;
    case MMA8451_PL_LRB:
      Serial.println("Landscape Right Back");
      break;
    case MMA8451_PL_LLF:
      Serial.println("Landscape Left Front");
      break;
    case MMA8451_PL_LLB:
      Serial.println("Landscape Left Back");
      break;
  }
  Serial.println();
  delay(500);
}

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
