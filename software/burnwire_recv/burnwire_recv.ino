#include <SPI.h>
#include <RH_RF95.h>

#define RF95_CS 4
#define RF95_G0 5
#define RF95_RST 6
#define BURN_PIN 2
#define BURN_MS 5000 // Burn time 5 seconds
 
// Singleton instance of the radio driver
RH_RF95 rf95(RF95_CS, RF95_G0);
 
uint8_t reply_start[] = "Burning the wire";  // msg back to ground
uint8_t reply_end[] = "Burn completed";  // msg back to ground

void setup() 
{
  Serial.begin(115200);
  Serial.println("Initializing");

  // Disable radio reset
  pinMode(RF95_RST, OUTPUT);
  digitalWrite(RF95_RST, HIGH);

  if (!rf95.init()) {
    Serial.println("RF95 module initialization failed!");
    while (1);  // Halt if radio initialization fails
  }

  rf95.setFrequency(915.0);  // Set frequency 
  
  pinMode(BURN_PIN, OUTPUT);  // Set the burn pin as output
  digitalWrite(BURN_PIN, LOW);  // burn pin is initially off

  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH); // LED on xiao is inverted

  Serial.println("T-Sat isready to receive messages.");
}

void loop()
{
  uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
  uint8_t len = sizeof(buf);

  // Wait for a message from Ground Station
  if (rf95.waitAvailableTimeout(5000)) { 
    //Checks if message was received 
    if (rf95.recv(buf, &len)) 
    {
      // Received a message, print it out and trigger burn action
      Serial.print("Received message: ");
      Serial.println((char*)buf);  // Print the received message

      if (strcmp((char*)buf, "BurnWire") == 0) 
      {
        // Message is "BurnWire", trigger LED and send response
        rf95.send(reply_start, sizeof(reply_start));

        // Burn!
        digitalWrite(BURN_PIN, HIGH);
        digitalWrite(LED_BUILTIN, LOW);

        Serial.println("Burn wire is ON (burn begin)");

        // Delay for burn duration
        delay(BURN_MS);

        // Turn off the burn wire
        digitalWrite(BURN_PIN, LOW);
        digitalWrite(LED_BUILTIN, HIGH);
        Serial.println("Burn wire is OFF (burn complete)");

        // Send response to Ground Station
        Serial.println("Sending reply: 'Burning the wire...'");
        rf95.send(reply_end, sizeof(reply_end));
      }
    } 
    else
    {
      Serial.println("Receive failed");
    }
  } 
  else 
  {
    Serial.println("No message received from Ground Station");
  }
}
