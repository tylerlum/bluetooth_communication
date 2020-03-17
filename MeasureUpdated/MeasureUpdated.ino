#include <Wire.h>
#include <VL53L0X.h>

// Sensors
VL53L0X platformSensor;
VL53L0X baseSensor;

// Pins
const int PWM_PIN = 10;
const int DIRECTION_UP_PIN = 9;
const int PLATFORM_SENSOR_XSHUT_PIN = 5;
const int BASE_SENSOR_XSHUT_PIN = 4;

const int PLATFORM_SENSOR_FAILURE_PIN = 6;
const int BASE_SENSOR_FAILURE_PIN = 7;

const int UPPER_LIMIT_SWITCH_PIN = 2;
const int LOWER_LIMIT_SWITCH_PIN = 3;

// Timing
unsigned long lastTime = millis();
const int AVERAGING_TIME_MS = 100;
long platformSensorDistanceAccumulation = 0;
long baseSensorDistanceAccumulation = 0;
int countLoops = 0;

void setupSensors() {
  // Turn off sensors using XSHUT pins
  pinMode(PLATFORM_SENSOR_XSHUT_PIN, OUTPUT);
  pinMode(BASE_SENSOR_XSHUT_PIN, OUTPUT);
  digitalWrite(PLATFORM_SENSOR_XSHUT_PIN, LOW);
  digitalWrite(BASE_SENSOR_XSHUT_PIN, LOW);

  // Setup I2C communication
  delay(500);
  Wire.begin();

  // Setup Serial for prints
  Serial.begin(9600);

  // Sensor 1 setup
  pinMode(PLATFORM_SENSOR_XSHUT_PIN, INPUT);  // Turn on sensor. Can't set OUTPUT HIGH b/c XSHUT can't handle 5V
  delay(150);
  platformSensor.init(true);
  delay(100);
  platformSensor.setAddress((uint8_t)22);

  // Sensor 2 setup
  pinMode(BASE_SENSOR_XSHUT_PIN, INPUT);  // Turn on sensor. Can't set OUTPUT HIGH b/c XSHUT can't handle 5V
  delay(150);
  baseSensor.init(true);
  delay(100);
  baseSensor.setAddress((uint8_t)25);

  platformSensor.setTimeout(500);
  baseSensor.setTimeout(500);
}

void setup()
{
  setupSensors();

  // Sensor failure pins 
  digitalWrite(PLATFORM_SENSOR_FAILURE_PIN, HIGH);
  digitalWrite(BASE_SENSOR_FAILURE_PIN, HIGH);
  pinMode(PLATFORM_SENSOR_FAILURE_PIN, OUTPUT);
  pinMode(BASE_SENSOR_FAILURE_PIN, OUTPUT);
  digitalWrite(PLATFORM_SENSOR_FAILURE_PIN, HIGH);
  digitalWrite(BASE_SENSOR_FAILURE_PIN, HIGH);

  // Limit switch pins
  pinMode(UPPER_LIMIT_SWITCH_PIN, INPUT_PULLUP);
  pinMode(LOWER_LIMIT_SWITCH_PIN, INPUT_PULLUP);

}

void checkAddresses() {
  byte count = 0;


  for (byte i = 1; i < 30; i++)
  {
    Wire.beginTransmission (i);
    if (Wire.endTransmission () == 0)
    {
      count++;
      delay (1);  
    }
  }
}

void loop()
{
  if (millis() - lastTime < AVERAGING_TIME_MS) {
    platformSensorDistanceAccumulation += (platformSensor.readRangeSingleMillimeters());
    baseSensorDistanceAccumulation += (baseSensor.readRangeSingleMillimeters());
    countLoops++;
  }
  else {
    long platformSensorDistance = platformSensorDistanceAccumulation / countLoops;
    long baseSensorDistance = baseSensorDistanceAccumulation / countLoops;
    checkAddresses();
  
    // Sensor 1
    if (!platformSensor.timeoutOccurred())
    {
      Serial.print("PlatformSensor: ");
      Serial.print(platformSensorDistance);
    } else {
      Serial.println("TIMEOUT 1");
      digitalWrite(PLATFORM_SENSOR_FAILURE_PIN, LOW);
    }
  
    // Sensor 2
    if (!baseSensor.timeoutOccurred())
    {
      Serial.print(" Base Sensor: ");
      Serial.print(baseSensorDistance);
    } else {
      Serial.println("TIMEOUT 2");
      digitalWrite(BASE_SENSOR_FAILURE_PIN, LOW);
    }

    Serial.print(" UPPER_LIMIT_SWITCH: ");   
    Serial.print(digitalRead(UPPER_LIMIT_SWITCH_PIN));
    Serial.print(" LOWER_LIMIT_SWITCH: ");   
    Serial.println(digitalRead(LOWER_LIMIT_SWITCH_PIN));
    platformSensorDistanceAccumulation = 0;
    baseSensorDistanceAccumulation = 0;
    countLoops = 0;
    lastTime = millis();
  }
}
