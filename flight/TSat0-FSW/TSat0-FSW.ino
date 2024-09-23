int LED_BUILTIN = 2;

// required for MMA8451----------------
#include <Wire.h>
#include <Adafruit_MMA8451.h>
#include <Adafruit_Sensor.h>
Adafruit_MMA8451 mma = Adafruit_MMA8451();

// required for BMP390-----------------
#include "Adafruit_BMP3XX.h"
#define SEALEVELPRESSURE_HPA (1013.25)
Adafruit_BMP3XX bmp;

// required for OLED display-----------
//#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
Adafruit_SSD1306 display(128, 64, &Wire, -1); //select reset pin (just set to any unused pin)

// required for CAM and SD-------------
#include "Arducam_Mega.h"
#include "Arducam/Platform.h"
#include "SPI.h"
#include "FS.h"
#include "SD.h"
#define CAM_CS 17
#define SCK 14
#define MISO 12
#define MOSI 13
#define CS 15
#define BUFFER_SIZE 0xff
#define PIC_COUNT 5
Arducam_Mega myCAM(CAM_CS);
//uint8_t count = 1;
char name[20] = {0};
File outFile;
uint8_t imageDataPrev = 0;
uint8_t imageData = 0;
uint8_t headFlag = 0;
unsigned int i = 0;
uint8_t imageBuff[BUFFER_SIZE] = {0};

File myFile;

// Function initializations:
void recordMMA();
void recordBMP();
void BMP_altitude_setup();
void take_image_save();

// Globals

int TEST_MODE = 1; //when == 1, test mode is on (for troubleshooting/testing)

float LOCAL_P = 0;

int FILENUM_CAM = 1; //camera var
int FILENUM_DATA = 1; //camera var

String dataString; //string to write to file
int fileIterator = 0; //filename number 
String fileName = "/EC-" + String(fileIterator) + ".txt"; //filename


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
    
    Serial.begin(9600);

    if(TEST_MODE == 1) {
        // setup OLED
        display.begin(SSD1306_SWITCHCAPVCC, 0x3c); // i2c address
        display.setTextSize(2);
        display.setTextColor(WHITE);
    }
    
	
    // setup CAM
    myCAM.begin();

    //setup SD
    if (!SD.begin(CS)) {
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
	
	
	
    // Lower SPI speed for stability
    SPI.setClockDivider(SPI_CLOCK_DIV2);

    // Delay to stop first image from being green
    arducamDelayMs(500);
    /*
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
    */
    // setup BMP
    if (!bmp.begin_I2C()) {   // hardware I2C mode, can pass in address & alt Wire
        Serial.println("Could not find a valid BMP3 sensor, check wiring!");
        while (1);
    }
    // configure BMP
    // Set up oversampling and filter initialization
    bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
    bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);
    bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
    bmp.setOutputDataRate(BMP3_ODR_50_HZ);

    // configure altitude relative to ground (aka. height on power-on)
    BMP_altitude_setup();

}

void loop() {
    
    // LED off once setup complete
    digitalWrite(LED_BUILTIN, LOW);

    
    
    
    if (continuousLoopTime > 10000) {
        
        take_image_save();
        continuousLoopTime = 0;
        
        
    }
    time1 = millis();
    

    if(TEST_MODE == 1) {
        // display code:
        display.clearDisplay();
        display.drawRoundRect(0, 0, 127, 63, 8, WHITE);
        display.setRotation(2);
        display.setCursor(20,20);
    }
    
  
    
    
    //float alt;
    alt = bmp.readAltitude(LOCAL_P);
    

    if(TEST_MODE == 1) {
        //Serial.print("Current altitude = ");
        //Serial.print(alt);
        //Serial.println(" m");
      
        // more display code:
        display.print(alt);
        display.print(" m");
        display.display();
    }

    // function call: read temp. save to global
    recordBMP();

    // function call: read gyro. save to global

    // if alt = going_up_alt or greater than: parachute_auth_flag = 1

    // if alt = going_down_alt or less than AND parachute_auth_flag = 1: deploy parachute

    // record data
    record_data_save();

    time2 = millis();
    singleMainLoopTime = time2 - time1;
    continuousLoopTime += singleMainLoopTime;
    
}



void parachute_trigger_check() {
	
	
	
}

void BMP_altitude_setup() {
    //double T, P;
    for(int i = 0; i < 4; i++) {
      if (! bmp.performReading()) {
        Serial.println("Failed to perform reading :(");
        return;
      }

      delay(2000);

      LOCAL_P = bmp.pressure / 100.0; //pressure in hPa
    
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
    dataString = String(float(millis())) + ", " + String(temperature) + ", " + String(pressure) + ", " + String(alt); // NOT COMPLETE
    
    if(myFile) {
        Serial.println("writing to text file successfully");
        myFile.println(dataString);
        myFile.close(); //good
        Serial.println("Done...");
    }
    else {
      Serial.println("error opening .txt");
    }
    
    
}

void take_image_save() {
    Serial.println("START take_image_save");
    
    
    digitalWrite(LED_BUILTIN, HIGH);

    /*
    // Stop after 5 images
    if (count > PIC_COUNT) {
        SD.end();
        Serial.println(F("SD card safe to remove"));
        while (1);
    }
    */
    
    // Take picture
    myCAM.takePicture(CAM_IMAGE_MODE_WQXGA2,CAM_IMAGE_PIX_FMT_JPG);
    
    // Save image
    while (myCAM.getReceivedLength()) {
        // Store current and previous byte
        imageDataPrev = imageData;
        imageData = myCAM.readByte();

        // Write data to buffer
        if (headFlag == 1) {
            imageBuff[i++]=imageData;
            // When buffer is full, write to file
            if (i >= BUFFER_SIZE) {
                outFile.write(imageBuff, i);
                i = 0;
            }
        }
        // Initialize file on JPEG file start (0xFFD8)
        if (imageDataPrev == 0xff && imageData == 0xd8) {
            headFlag = 1;
            sprintf(name,"/%d.jpg", FILENUM_CAM);

            // ensure filename does not already exist
            while(SD.exists(name)) {
              FILENUM_CAM++;
              sprintf(name, "/%d.jpg", FILENUM_CAM);
            }
            
            //count++;
            
            //Serial.print(F("Saving image..."));
            outFile = SD.open(name, "w");
            if (!outFile) {
                Serial.println(F("Error"));
                Serial.println(F("File open failed"));
                while (1);
            }
            imageBuff[i++]=imageDataPrev;
            imageBuff[i++]=imageData;
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
    if(TEST_MODE == 1) {
      if (! bmp.performReading()) {
          Serial.println("Failed to perform reading :(");
          return;
      }
    }
    
    
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
