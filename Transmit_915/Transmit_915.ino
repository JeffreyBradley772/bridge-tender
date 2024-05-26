// Transmit code at 915 Freq

#include <Adafruit_SleepyDog.h>
#include <SPI.h>
#include <RH_RF95.h>
#include <Wire.h> // Must include Wire library for I2C
#include "SparkFun_MMA8452Q.h" // Library for the MMA8452Q accelerometer
#include <math.h>

// Accelerometer instance
MMA8452Q accel;

// Structure to hold accelerometer readings
struct dataStruct {
  float stat;
  double angle;
} AccelReadings;

// Pin definitions for Feather32u4
#define RFM95_CS 8
#define RFM95_RST 4
#define RFM95_INT 7

// Frequency for the LoRa radio
#define RF95_FREQ 915.0

// Singleton instance of the radio driver
RH_RF95 rf95(RFM95_CS, RFM95_INT);

// Variables to store accelerometer data
float stat;
double angle, accel_x, accel_y, accel_z, accel_x2, accel_y2, accel_z2;

// Buffer to hold the serialized data
byte buf[sizeof(AccelReadings)] = {0};

void setup() {
  // Initialize the built-in LED and set it to high to indicate the device is awake
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);

  // Initialize the accelerometer data structure
  AccelReadings.stat = 0;
  AccelReadings.angle = 0;

  // Initialize RFM95 LoRa radio
  pinMode(RFM95_RST, OUTPUT);
  digitalWrite(RFM95_RST, HIGH);
  Wire.begin();
  Serial.begin(9600);
  while (!Serial) {
    delay(1);
  }

  delay(10);
  Serial.println("Feather LoRa TX Test!");

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
  rf95.setTxPower(13, false);

  // Initialize the accelerometer
  accel.begin();
}

void loop() {
  // Read accelerometer values twice with a delay
  accel_x = accel.getCalculatedX();
  accel_y = accel.getCalculatedY();
  accel_z = accel.getCalculatedZ();
  delay(1000);
  accel_x2 = accel.getCalculatedX();
  accel_y2 = accel.getCalculatedY();
  accel_z2 = accel.getCalculatedZ();

  // Calculate angles and difference
  double ang = atan(accel_y / accel_z);
  double ang2 = atan(accel_y2 / accel_z2);
  double angl1 = ang * (180.0 / 3.14);
  double angl2 = ang2 * (180.0 / 3.14);
  double dif = angl2 - angl1;

  // Print the angles for debugging
  Serial.println(angl1);
  delay(1000);
  Serial.println(dif);

  // Determine the status based on the angle and its change
  angle = angl1;
  if (angle < 10) {
    stat = 0;
  } else if (angle > 10 && angle < 80 && dif > 0) { // going up
    stat = 1;
  } else if (angle > 10 && angle < 80 && dif < 0) { // going down
    stat = 2;
  } else if (angle > 80) {
    stat = 3;
  }

  // Update the data structure with the current readings
  AccelReadings.angle = angle;
  AccelReadings.stat = stat;

  // Copy the data structure into the buffer
  byte size = sizeof(AccelReadings);
  memcpy(buf, &AccelReadings, size);

  // Transmit the data
  Serial.println("Transmitting...");
  delay(10);
  rf95.send(buf, size);
  Serial.println("Waiting for packet to complete...");
  delay(10);
  rf95.waitPacketSent();

  // Wait 1 second for a reply (could be longer...?)
  uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
  uint8_t len = sizeof(buf);
  Serial.println("Waiting for reply...");
  if (rf95.waitAvailableTimeout(1000)) {
    if (rf95.recv(buf, &len)) {
      Serial.print("Got reply from receiver: ");
      Serial.println((char*)buf);
      Serial.print("RSSI: ");
      Serial.println(rf95.lastRssi(), DEC);
    } else {
      Serial.println("Receive failed.");
    }
  } else {
    Serial.println("No reply...");
  }
}
