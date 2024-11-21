#include <ESP32Servo.h>

#define SERVO_PIN 4
#define SERVO_STOW_POS 110
#define SERVO_DEPLOY_POS 180

Servo servo;

void setup() {
  servo.attach(SERVO_PIN);
  servo.write(SERVO_STOW_POS);

  pinMode(LED_BUILTIN, OUTPUT);

  digitalWrite(LED_BUILTIN, HIGH);
  
  delay(2000);

  digitalWrite(LED_BUILTIN, LOW);

  delay(2000);

  servo.write(SERVO_DEPLOY_POS);
}

void loop() {}
