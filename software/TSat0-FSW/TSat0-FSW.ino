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
#include <SD.h>
#include <SPI.h>

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

#define CALIBRATION_COUNT 20             // how many measurements to average when determining initial altitude
#define CONSISTENT_READING_THRESHOLD 1   // how many measurements in a row need to agree to change state
#define ASCENT_ACCEL_THRESHOLD 20        // acceleration above which PREFLIGHT moves to ASCENT
#define MAX_PREFLIGHT_ALTITUDE 3         //20    // altitude where PREFLIGHT automatically switches to ASCENT
#define ALTITUDE_CHECK_DELAY 50          // ms between altitude checks
#define FREEFALL_ACCEL_THRESHOLD 4       // acceleration below which to switch from ASCENT to FREEFALL
#define PARACHUTE_DEPLOY_ALTITUDE 10     //80 // altutude to switch from FREEFALL to LANDING
#define ALTITUDE_CHANGE_FILTER_GAIN 0.8  // between 0 and 1, higher number means each measurement has lower impact on estimate
#define ACCEL_FILTER_GAIN 0.5

#define SERVO_STOW_POS 110
#define SERVO_DEPLOY_POS 180

// Define TEST_MODE to enable test mode
// #define TEST_MODE

// Peripheral globals
Adafruit_MMA8451 mma = Adafruit_MMA8451();
Adafruit_BMP3XX bmp;
Adafruit_SSD1306 display(128, 64, &Wire, -1);  // select reset pin (just set to any unused pin)
Arducam_Mega cam(CAM_CS);
Servo servo;

// Function definitions
void write_pic(Arducam_Mega &cam, File dest);
void calibrationRun();
void preflightRun();
void ascentRun();
void freefallRun();
void landingRun();
void checkAltitude(void *parameter);
void log_SD(int index);

// Globals
static TaskHandle_t check_altitude = NULL;
int pic_num = 0;
char base_dir[20];
float start_altitude = 0;
int calibration_count = 0;
int consistent_reading_count = 0;
float accel_estimate = 0;
float prev_accel_estimate = 0;
float absolute_altitude = 0;
float ground_altitude = 0;
char logpath[35];
int logindex = 0;

typedef enum {
  CALIBRATION,  // establishing altitude baseline
  PREFLIGHT,    // waiting for launch
  ASCENT,       // rising on balloon
  FREEFALL,     // detached from balloon
  LANDING,      // parachute deployed
} FlightState;

FlightState flight_state = CALIBRATION;

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

  // setup SD
  if (!SD.begin(SD_CS)) {
    Serial.println(F("Error"));
    Serial.println(F("Card mount failed"));
    while (1) {}
  }
  uint8_t cardType = SD.cardType();
  if (cardType == CARD_NONE) {
    Serial.println(F("Error"));
    Serial.println(F("No SD card attached"));
    while (1) {}
  }

  // setup MMA
  if (!mma.begin()) {
    Serial.println("Couldnt start MMA");
    while (1) {}
  }
  mma.setRange(MMA8451_RANGE_2_G);

  // setup BMP
  if (!bmp.begin_I2C()) {  // hardware I2C mode, can pass in address & alt Wire
    Serial.println("Couldn't start BMP");
    while (1) {}
  }
  // Set up oversampling and filter initialization
  bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
  bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);
  bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
  bmp.setOutputDataRate(BMP3_ODR_50_HZ);

  // Preload data into bmp
  bmp.performReading();

  // Pick base directory for this flight
  int i = 0;
  do {
    sprintf(base_dir, "/tsatlog%d", i);
    i++;
  } while (SD.exists(base_dir));
  SD.mkdir(base_dir);
  Serial.print("Data from this run stored in ");
  Serial.println(base_dir);


  sprintf(logpath, "%s/data.csv", base_dir);





  // Create task for sensor checks
  xTaskCreatePinnedToCore(
    checkAltitude,     // Function to call
    "Check Altitude",  // Task name
    4096,              // Memory allocated
    NULL,              // Parameters to pass to the function
    1,                 // Priority (higher number = higher priority)
    &check_altitude,   // Task handle
    0                  // Core
  );

  // Delay to stop first image from being green
  arducamDelayMs(500);

  // LED off once setup complete
  digitalWrite(LED_BUILTIN, LOW);
}

void loop() {}

// Periodically monitors sensor data and performs state switches
void checkAltitude(void *parameter) {

  TickType_t last_wake = xTaskGetTickCount();

  while (1) {
    /*
  Serial.print("absolute alt: ");
  Serial.println(absolute_altitude);
  Serial.print("start alt: ");
  Serial.println(start_altitude);
  Serial.print("alt: ");
  Serial.println(ground_altitude);
  */

    // Get altitude data
    bmp.performReading();
    absolute_altitude = bmp.readAltitude(SEALEVELPRESSURE_HPA);
    ground_altitude = absolute_altitude - start_altitude;

    // Get imu data
    mma.read();
    float accel_x = mma.x_g * SENSORS_GRAVITY_STANDARD;
    float accel_y = mma.y_g * SENSORS_GRAVITY_STANDARD;
    float accel_z = mma.z_g * SENSORS_GRAVITY_STANDARD;
    float accel = sqrtf(accel_x * accel_x + accel_y * accel_y + accel_z * accel_z);

    // Lowpass filter, smoothes out noisy data
    accel_estimate = (ACCEL_FILTER_GAIN * prev_accel_estimate) + (1 - ACCEL_FILTER_GAIN) * accel;
    prev_accel_estimate = accel_estimate;

    /*
    Serial.print("accel:");
    Serial.println(accel_estimate);
    */

    // Core state machine
    switch (flight_state) {
      case CALIBRATION:
        /* CALIBRATE: averages measurements to
       * determine a launch altitude. */
        calibrationRun();
        break;
      case PREFLIGHT:
        /* PRE-LAUNCH: waiting on the ground to be released. Detects
       * launch by breaking an altitude rate of change threshold
       * or passing a certain altitude.       - > ASCENT         */
        preflightRun();
        break;
      case ASCENT:
        /* ASCENT: Rising up to the extent of the tether and waiting
       * for the burn wire to trigger and free fall to begin.
       * Detects this by breaking an altitude rate of change
       * threshold (or accelerometer info)    - > FREEFALL       */
        ascentRun();
        break;
      case FREEFALL:
        /* FREE FALL: falling through the air and waiting to
       * hit the parachute deployment altitude.  - > LANDING      */
        freefallRun();
        break;
      case LANDING:
        /* LANDING: Deploy parachute. Final State. */
        landingRun();
        break;
    }

    log_SD(logindex);
    logindex++;

#ifdef TEST_MODE
    // display code:
    display.clearDisplay();
    display.drawRoundRect(0, 0, 128, 64, 8, WHITE);
    display.setRotation(2);
    display.setCursor(15, 3);
    if (flight_state != CALIBRATION) {
      display.setCursor(altitude >= 0 ? 22 : 10, 8);
      display.print(altitude);
    } else {
      display.setCursor(absolute_altitude >= 0 ? 22 : 10, 8);
      display.print(absolute_altitude);
    }
    display.print(" m");
    display.setCursor(altitude_change_estimate >= 0 ? 22 : 10, 28);
    display.print(altitude_change_estimate * 1000 / ALTITUDE_CHECK_DELAY);
    display.print(" m/s");

    display.setCursor(10, 48);
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

    vTaskDelayUntil(&last_wake, ALTITUDE_CHECK_DELAY / portTICK_PERIOD_MS);
  }
}


/* ==== UTILITY FUNCTIONS ==== */

void write_pic(Arducam_Mega &cam, File dest) {

  uint8_t count = 1;
  uint8_t prev_byte = 0;
  uint8_t cur_byte = 0;
  uint8_t head_flag = 0;
  unsigned int i = 0;
  uint8_t image_buf[PIC_BUFFER_SIZE] = { 0 };

  while (cam.getReceivedLength()) {
    // Store current and previous byte
    prev_byte = cur_byte;
    cur_byte = cam.readByte();

    // Write data to buffer
    if (head_flag == 1) {
      image_buf[i++] = cur_byte;
      // When buffer is full, write to file
      if (i >= PIC_BUFFER_SIZE) {
        dest.write(image_buf, i);
        i = 0;
      }
    }
    // Initialize file on JPEG file start (0xFFD8)
    if (prev_byte == 0xff && cur_byte == 0xd8) {
      head_flag = 1;
      // sprintf(name,"/%d.jpg", count);
      count++;
      // Serial.print(F("Saving image..."));
      image_buf[i++] = prev_byte;
      image_buf[i++] = cur_byte;
    }
    // Close file on JPEG file ending (0xFFD9)
    if (prev_byte == 0xff && cur_byte == 0xd9) {
      // headFlag = 0;
      dest.write(image_buf, i);
      // i = 0;
      dest.close();
      Serial.println(F("Done"));
      break;
    }
  }
}

void log_SD(int index) {

  File dataStorage = SD.open(logpath, FILE_APPEND);

  float accelX = mma.x_g * SENSORS_GRAVITY_STANDARD;
  float accelY = mma.y_g * SENSORS_GRAVITY_STANDARD;
  float accelZ = mma.z_g * SENSORS_GRAVITY_STANDARD;

  if (dataStorage) {
    dataStorage.print(index);
    dataStorage.print(",");
    dataStorage.print(bmp.temperature);
    dataStorage.print(",");
    dataStorage.print(bmp.pressure);
    dataStorage.print(",");
    dataStorage.print(accelX);
    dataStorage.print(",");
    dataStorage.print(accelY);
    dataStorage.print(",");
    dataStorage.println(accelZ);
    dataStorage.close();
  } else {
    Serial.println("Error opening file");
  }
}

/* ================================= */

/* ==== STATE MACHINE FUNCTIONS ==== */

void calibrationRun() {
  // average several readings for baseline then move to PRELAUNCH
  start_altitude += absolute_altitude;
  calibration_count++;

  Serial.print("Calibrating altitude ");
  Serial.print(calibration_count);
  Serial.print("/");
  Serial.println(CALIBRATION_COUNT);

  if (calibration_count >= CALIBRATION_COUNT) {
    start_altitude /= CALIBRATION_COUNT;
    flight_state = PREFLIGHT;

    Serial.print("Starting altitude: ");
    Serial.println(start_altitude);
    Serial.println("Calibration complete, moving to PREFLIGHT");
  }
}

void preflightRun() {
  // move to ASCENT if acceleration is or altitude above threshold

  if (ground_altitude >= MAX_PREFLIGHT_ALTITUDE) {
    flight_state = ASCENT;
    consistent_reading_count = 0;

    Serial.println("Max preflight altitude exceeded, moving to ASCENT");
  } else if (accel_estimate >= ASCENT_ACCEL_THRESHOLD) {
    consistent_reading_count++;
    if (consistent_reading_count >= CONSISTENT_READING_THRESHOLD) {
      flight_state = ASCENT;
      consistent_reading_count = 0;

      Serial.println("Acceleration threshold met, moving to ASCENT");
    }
  } else {
    consistent_reading_count = 0;
  }
}

void ascentRun() {
  // move to FREEFALL if acceleration is close to 0
  if (accel_estimate <= FREEFALL_ACCEL_THRESHOLD) {
    consistent_reading_count++;
    if (consistent_reading_count >= CONSISTENT_READING_THRESHOLD) {
      flight_state = FREEFALL;

      digitalWrite(LED_BUILTIN, HIGH);

      consistent_reading_count = 0;

      Serial.println("Acceleration threshold met, moving to FREEFALL");
    }
  } else {
    consistent_reading_count = 0;
  }
}

void freefallRun() {
  // move to LANDING if below deployment altitude
  if (ground_altitude < PARACHUTE_DEPLOY_ALTITUDE) {
    flight_state = LANDING;

    Serial.println("Parachute deployment altitude reached, moving to LANDING");
    servo.write(SERVO_DEPLOY_POS);
  }
}

void landingRun() {
  // Deploy parachute, no more state changes
}

/* ================================= */