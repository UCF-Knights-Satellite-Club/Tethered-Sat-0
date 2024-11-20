#include <Adafruit_MMA8451.h>

Adafruit_MMA8451 mma = Adafruit_MMA8451();

float prev_estimate = 0;

void setup() {
  Serial.begin(115200);

  // setup MMA
  if (!mma.begin()) {
    Serial.println("Couldnt start MMA");
    while (1) {};
  }
  mma.setRange(MMA8451_RANGE_2_G);
}

void loop() {
  mma.read();
  int x = mma.x;
  float gx = (float) x * 2.0 / 8192.0 * SENSORS_GRAVITY_STANDARD;
  int y = mma.y;
  float gy = (float)y * 2.0 / 8192.0 * SENSORS_GRAVITY_STANDARD;
  int z = mma.z;
  float gz = (float)z * 2.0 / 8192.0 * SENSORS_GRAVITY_STANDARD;

  float magnitude = sqrtf(gx*gx + gy*gy + gz*gz);
  float estimate = (prev_estimate * 0.8) + (magnitude * 0.2);
  prev_estimate = magnitude;

  Serial.print("x:"); Serial.println(mma.x_g);
  Serial.print("y:"); Serial.println(mma.y_g);
  Serial.print("z:"); Serial.println(mma.z_g);
  Serial.print("accel:"); Serial.println(estimate);

  delay(100);
}
