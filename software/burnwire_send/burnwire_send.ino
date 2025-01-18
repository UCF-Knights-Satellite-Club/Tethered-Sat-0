//Ground ESP32
#include <SPI.h>
#include <RH_RF95.h>

#define RF95_CS 5
#define RF95_G0 21
#define RF95_RST 4
#define BUTTON_PIN 2  // Pin where the button is connected

// Singleton instance of the radio driver
RH_RF95 rf95(RF95_CS, RF95_G0);

// Button Pin and Variables
int buttonValue = 0;
int lastButtonValue = 0;

// Message to be sent when the button is pressed
uint8_t burn[] = "BurnWire";

void setup() {
  Serial.begin(115200);
  Serial.println("Initializing");

  // Disable radio reset
  pinMode(4, OUTPUT);
  digitalWrite(4, HIGH);

  pinMode(BUTTON_PIN, INPUT);  // Set button pin as input

  if (!rf95.init()) {
    Serial.println("init failed");
    while (1)
      ;  // if radio init fails then it stops
  }

  rf95.setFrequency(915.0);  // Set frequency
  //rf95.setTxPower(20, false);

  Serial.println("Ground Station Ready");
}

void loop() {
  buttonValue = digitalRead(BUTTON_PIN);

  // Check if button is pressed
  if (buttonValue == HIGH && lastButtonValue == LOW) {
    // Button was pressed, send "BurnWire" message
    Serial.println("Sending 'BurnWire' message...");
    rf95.send(burn, sizeof(burn));  // Send the message
    rf95.waitPacketSent();          // Wait for transmission to complete

    // Print confirmation of message
    Serial.println("Message sent to T-Sat: 'BurnWire'");

    //Turn on LED to indicate its sending
    //digitalWrite(led, HIGH);
    delay(5000);  // Keep LED on for 5 secs.
    //digitalWrite(led, LOW);

    //Wait for the reply
    uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
    uint8_t len = sizeof(buf);

    if (rf95.waitAvailableTimeout(3000))  // Wait for a reply for 3 seconds
    {
      if (rf95.recv(buf, &len)) {
        Serial.print("Received reply: ");
        Serial.println((char*)buf);
      } else {
        Serial.println("Receive failed");  // signal detected but fail to rec
      }
    } else {
      Serial.println("No reply received");  //no signal at all
    }
  }

  // Store current state of button
  lastButtonValue = buttonValue;


  // Add delay for button press
  if (rf95.waitAvailableTimeout(50))  // Wait for a reply
  {
    uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
    uint8_t len = sizeof(buf);
    if (rf95.recv(buf, &len)) {
      Serial.print("Received reply: ");
      Serial.println((char*)buf);
    } else {
      Serial.println("Receive failed");  // signal detected but fail to rec
    }
  }
}
