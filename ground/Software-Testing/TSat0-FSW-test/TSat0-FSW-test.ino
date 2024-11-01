

//required for MMA8451----------------
#include <Wire.h>
#include <Adafruit_MMA8451.h>
#include <Adafruit_Sensor.h>
#include <Arduino.h>

Adafruit_MMA8451 mma = Adafruit_MMA8451();

//required for BMP390-----------------
#include "Adafruit_BMP3XX.h"
#define SEALEVELPRESSURE_HPA (1013.25)
Adafruit_BMP3XX bmp;

static TaskHandle_t check_altitude = NULL;

// Function initializations:
void printMMA();
void printBMP();
void checkAltitude();

void setup() {

    // Function to create a new task, must be run in setup
    // Reference: (https://www.youtube.com/watch?v=95yUbClyf3E)
    xTaskCreatePinnedToCore(
      checkAltitude,    // Function to call
      "Check Altitude", // Task name
      1024,             // Memory allocated
      NULL,             // Parameters to pass to the function
      1,                // Priority (higher number = higher priority)
      &check_altitude,  // Task handle
      0                 // Core
    );

    Serial.begin(9600);
  
    //Serial.println("Adafruit MMA8451 test!");
  
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
    if (!bmp.begin_I2C()) {   // hardware I2C mode, can pass in address & alt Wire
        Serial.println("Could not find a valid BMP3 sensor, check wiring!");
        while (1);
    }
    // configure MMA
    // Set up oversampling and filter initialization
    bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
    bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);
    bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
    bmp.setOutputDataRate(BMP3_ODR_50_HZ);
}

void loop() {
    printMMA();
    printBMP();
}


void printMMA() {

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

void printBMP() {
    if (! bmp.performReading()) {
        Serial.println("Failed to perform reading :(");
        return;
    }
    Serial.print("Temperature = ");
    Serial.print(bmp.temperature);
    Serial.println(" *C");

    Serial.print("Pressure = ");
    Serial.print(bmp.pressure / 100.0);
    Serial.println(" hPa");

    Serial.print("Approx. Altitude = ");
    Serial.print(bmp.readAltitude(SEALEVELPRESSURE_HPA));
    Serial.println(" m");

    Serial.println();
    delay(2000);
}

// A parameter of type (void*) is needed to prevent error when creating the task
void checkAltitude(void* parameter) {
  // While loop needed to run forever
  while (1) {
    // Check the altitude and do whatever
    Serial.println("CHECKING THE ALTITUDE");
    // Delay this task for 500 ms
    vTaskDelay(500 / portTICK_PERIOD_MS);
  }
}