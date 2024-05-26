// Receive code at 915 Freq

#include <SPI.h>
#include <RH_RF95.h>

// Pin definitions for Feather32u4
#define RFM95_CS 8
#define RFM95_RST 4
#define RFM95_INT 7

#if defined(ESP8266)
  // Pin definitions for ESP8266
  #define RFM95_CS  2
  #define RFM95_RST 16
  #define RFM95_INT 15
#elif defined(ESP32)
  // Pin definitions for ESP32
  #define RFM95_RST 27
  #define RFM95_CS  33
  #define RFM95_INT 12
#elif defined(NRF52)
  // Pin definitions for nRF52832
  #define RFM95_RST 7
  #define RFM95_CS  11
  #define RFM95_INT 31
#elif defined(TEENSYDUINO)
  // Pin definitions for Teensy 3.x
  #define RFM95_RST 9
  #define RFM95_CS  10
  #define RFM95_INT 4
#endif

// Frequency for the LoRa radio, must match the transmitter's frequency
#define RF95_FREQ 915.0

// Singleton instance of the radio driver
RH_RF95 rf95(RFM95_CS, RFM95_INT);

// Pin for the built-in LED
#define LED 13

// Structure to hold the received data
struct dataStruct {
  float stat;
  double angle;
};

void setup() {
  // Initialize the built-in LED
  pinMode(LED, OUTPUT);
  pinMode(RFM95_RST, OUTPUT);
  digitalWrite(RFM95_RST, HIGH);

  // Initialize serial communication
  Serial.begin(115200);
  while (!Serial) {
    delay(1);
  }
  delay(100);

  Serial.println("Feather LoRa RX Test!");

  // Manual reset of the radio module
  digitalWrite(RFM95_RST, LOW);
  delay(10);
  digitalWrite(RFM95_RST, HIGH);
  delay(10);

  // Initialize the LoRa radio
  while (!rf95.init()) {
    Serial.println("LoRa radio init failed");
    while (1);
  }
  Serial.println("LoRa radio init OK!");

  // Set the frequency for the radio
  if (!rf95.setFrequency(RF95_FREQ)) {
    Serial.println("setFrequency failed");
    while (1);
  }
  Serial.print("Set Freq to: ");
  Serial.println(RF95_FREQ);

  // Set the transmitter power
  rf95.setTxPower(23, false);
}

void loop() {
  if (rf95.available()) {
    // Buffer to hold the received data
    uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
    uint8_t len = sizeof(buf);

    // Receive the data
    if (rf95.recv(buf, &len)) {
      if (len == sizeof(dataStruct)) {
        // Print the received buffer for debugging
        RH_RF95::printBuffer("Received: ", buf, len);

        // Copy the received data into the data structure
        dataStruct receivedData;
        memcpy(&receivedData, buf, sizeof(dataStruct));

        // Print the received data
        Serial.print("Stat: ");
        float status = receivedData.stat;
        Serial.println(status);
        if (status == 0) {
          Serial.println("DOWN.");
        } else if (status == 1) {
          Serial.println("GOING UP");
        } else if (status == 2){
          Serial.println("GOING DOWN.");
        }else if (status == 3){
          Serial.println("BRIDGE IS UP.");
        }
        
        Serial.print("Angle: ");
        Serial.println(receivedData.angle);
        
        // Blink the LED to indicate reception
        digitalWrite(LED, HIGH);
      } else {
        Serial.println("Received data does not match expected size.");
      }

      // Print the RSSI (Received Signal Strength Indicator)
      Serial.print("RSSI: ");
      Serial.println(rf95.lastRssi(), DEC);

      // Send a reply
      uint8_t data[] = "GOT IT!";
      rf95.send(data, sizeof(data));
      rf95.waitPacketSent();
      Serial.println("Sent a reply.");
      digitalWrite(LED, LOW); // Turn off LED after replying
    } else {
      Serial.println("Receive failed.");
    }
  }
}
