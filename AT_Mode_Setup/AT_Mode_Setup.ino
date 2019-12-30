#include <SoftwareSerial.h>

// Pins
const int BT_RX_PIN = 3;
const int BT_TX_PIN = 2;

// Global variables
const int BAUD_RATE = 38400;  // Needs to match the baud rate currently set on the Bluetooth module. If you don't know, try different common values.
// const int BAUD_RATE = 9600;

// Bluetooth Serial
SoftwareSerial BTserial(BT_TX_PIN, BT_RX_PIN); // Arduino RX | Arduino TX

void setup() {
  // Serial port setup
  Serial.begin(BAUD_RATE);
  BTserial.begin(BAUD_RATE); 

  // Show user that the setup is complete
  Serial.println("Enter AT commands:");
}

void loop() {
  // Keep reading from Bluetooth module and send to Arduino Serial Monitor
  if (BTserial.available()) { Serial.write(BTserial.read()); }

  // Keep reading from Arduino Serial Monitor and send to Bluetooth module
  if (Serial.available()) { BTserial.write(Serial.read()); }
}
