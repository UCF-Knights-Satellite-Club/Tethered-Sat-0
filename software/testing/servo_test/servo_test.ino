#include <ESP32Servo.h>

#define SERVO_PIN 4
#define SERVO_STOW_POS 180
#define SERVO_DEPLOY_POS 120

Servo servo;

void setup() {
  servo.attach(SERVO_PIN);
  servo.write(SERVO_STOW_POS);

  delay(2000);

  servo.write(SERVO_DEPLOY_POS);
}

void loop() {}
