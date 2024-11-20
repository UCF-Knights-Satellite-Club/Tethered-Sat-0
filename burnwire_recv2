// rf95_client.pde
// -*- mode: C++ -*-
// Example sketch showing how to create a simple messageing client
// with the RH_RF95 class. RH_RF95 class does not provide for addressing or
// reliability, so you should only use RH_RF95 if you do not need the higher
// level messaging abilities.
// It is designed to work with the other example rf95_server
// Tested with Anarduino MiniWirelessLoRa, Rocket Scream Mini Ultra Pro with
// the RFM95W, Adafruit Feather M0 with RFM95
//Ground ESP32
#include <SPI.h>
#include <RH_RF95.h>

// Singleton instance of the radio driver
RH_RF95 rf95(5, 21);
//RH_RF95 rf95(5, 2); // Rocket Scream Mini Ultra Pro with the RFM95W
//RH_RF95 rf95(8, 3); // Adafruit Feather M0 with RFM95 
// Need this on Arduino Zero with SerialUSB port (eg RocketScream Mini Ultra Pro)
//#define Serial SerialUSB

// Button Pin and Variables
int ButtonValue = 0; 
int lastButtonValue = 0;
int Button = 3;          // Pin where the button is connected
int led = LED_BUILTIN;   // Built-in LED ESP32

// Message to be sent when the button is pressed
uint8_t burn[] = "BurnWire"; 

void setup() 
{
  //If not using external resistor INPUT_PULLUP
  pinMode(Button, INPUT);  // Set button pin as input
  pinMode(led, OUTPUT);    // Set LED pin as output
  
  Serial.begin(115200);
  while (!Serial) ;  // Wait for serial port to be available
  
  if (!rf95.init()) {
    Serial.println("init failed");
    while (1);  // if radio inti fails then it stops  
  }

  rf95.setFrequency(915.0);  // Set frequency 
  Serial.println("Ground Station Ready");
  // The default transmitter power is 13dBm, using PA_BOOST.
  // If you are using RFM95/96/97/98 modules which uses the PA_BOOST transmitter pin, then 
  // you can set transmitter powers from 2 to 20 dBm:
//  rf95.setTxPower(20, false);
  // If you are using Modtronix inAir4 or inAir9, or any other module which uses the
  // transmitter RFO pins and not the PA_BOOST pins
  // then you can configure the power transmitter power for 0 to 15 dBm and with useRFO true. 
  // Failure to do that will result in extremely low transmit powers.
//  rf95.setTxPower(14, true);
}

void loop()
{
  ButtonValue = digitalRead(Button);

  // Check if button is pressed
  if (ButtonValue == HIGH && lastButtonValue == LOW) {
    // Button was pressed, send "BurnWire" message
    Serial.println("Sending 'BurnWire' message...");
    rf95.send(burn, sizeof(burn));  // Send the message
    rf95.waitPacketSent();  // Wait for transmission to complete

    //Turn on LED to indicate its sending 
    digitalWrite(led, HIGH); 
    delay(5000);               // Keep LED on for 5 secs.
    digitalWrite(led, LOW);   

    // Print confirmation of message
    Serial.println("Message sent to T-Sat: 'BurnWire'");
  }

  // Store current state of button
  lastButtonValue = ButtonValue;

  //Wait for the reply 
  uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
  uint8_t len = sizeof(buf);
  
  if (rf95.waitAvailableTimeout(3000)) // Wait for a reply for 3 seconds
  {  
    if (rf95.recv(buf, &len)) {
      Serial.print("Received reply: ");
      Serial.println((char*)buf);
    } 
    else 
    {
      Serial.println("Receive failed"); // signal detected but fail to rec
    }
  } 
  else 
  {
    Serial.println("No reply received is rf95_server running?"); //no signal at all
  }
  delay(50);  // Add delay for button press
}
