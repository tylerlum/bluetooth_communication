#include <Wire.h>
#include <VL53L0X.h>

// Sensor object
VL53L0X sensor;

// Pins
const int LED_PIN = 10;

// Global variables
int valueToSend = 0;

void setup() {
  // Setup Serial port
  Serial.begin(9600);

  // Setup sensor
  Wire.begin();
  sensor.init();
  sensor.setTimeout(500);
  sensor.startContinuous();
}

void loop() {
  // Read sensor
  valueToSend = sensor.readRangeContinuousMillimeters();

  // Scale value. AnalogWrite is [0,255]. Sensor can give [0,65535]. Usual valid sensor values are [0,700].
  valueToSend = valueToSend > 800 ? 255 : valueToSend / 4;
  
  // Write to Serial port and LED
  Serial.write(valueToSend);
  analogWrite(LED_PIN, valueToSend);

  // Throttle loop
  delay(10);
}
