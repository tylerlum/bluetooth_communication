#include <Wire.h>
#include <VL53L0X.h>

// Sensor object
VL53L0X sensor;

// Pins
const int LED_PIN = 10;

// Global variables
int valueToSend = 0;
double measuredValue = 0;

int count = 0;
double accumulation = 0;

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

  if (count == 20) {
    Serial.println(accumulation / count);
    count = 0;
    accumulation = 0;
  } else {
    count++;
    accumulation += sensor.readRangeContinuousMillimeters();
  }
  
  // Read sensor

  // Scale value. AnalogWrite is [0,255]. Sensor can give [0,65535]. Usual valid sensor values are [0,700].
////  valueToSend = measuredValue > 800 ? 255 : measuredValue / 4;
//  
//  // Write to Serial port and LED
//  // Serial.write(valueToSend);
//  if (millis() - lastMeasurementTime > 100) {
//    
//  } else {
//
//  }
//  Serial.println(measuredValue);
////  analogWrite(LED_PIN, valueToSend);
//
//  // Throttle loop
//  delay(10);
}
