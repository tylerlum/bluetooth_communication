#include <Wire.h>
#include <VL53L0X.h>

// Sensors
VL53L0X sensor1;
VL53L0X sensor2;

// Pins
const int PWM_PIN = 10;
const int DIRECTION_UP_PIN = 9;
const int SENSOR_1_XSHUT_PIN = 5;
const int SENSOR_2_XSHUT_PIN = 4;

const int SENSOR_1_FAILURE_PIN = 6;
const int SENSOR_2_FAILURE_PIN = 7;

const int UPPER_LIMIT_SWITCH_PIN = 2;
const int LOWER_LIMIT_SWITCH_PIN = 3;

// Timing
unsigned long lastTime = millis();
const int AVERAGING_TIME_MS = 100;
long upperSensorDistanceAccumulation = 0;
long lowerSensorDistanceAccumulation = 0;
int countLoops = 0;

void setupSensors() {
  // Turn off sensors using XSHUT pins
  pinMode(SENSOR_1_XSHUT_PIN, OUTPUT);
  pinMode(SENSOR_2_XSHUT_PIN, OUTPUT);
  digitalWrite(SENSOR_1_XSHUT_PIN, LOW);
  digitalWrite(SENSOR_2_XSHUT_PIN, LOW);

  // Setup I2C communication
  delay(500);
  Wire.begin();

  // Setup Serial for prints
  Serial.begin(9600);

  // Sensor 1 setup
  pinMode(SENSOR_1_XSHUT_PIN, INPUT);  // Turn on sensor. Can't set OUTPUT HIGH b/c XSHUT can't handle 5V
  delay(150);
  sensor1.init(true);
  delay(100);
  sensor1.setAddress((uint8_t)22);

  // Sensor 2 setup
  pinMode(SENSOR_2_XSHUT_PIN, INPUT);  // Turn on sensor. Can't set OUTPUT HIGH b/c XSHUT can't handle 5V
  delay(150);
  sensor2.init(true);
  delay(100);
  sensor2.setAddress((uint8_t)25);

  sensor1.setTimeout(500);
  sensor2.setTimeout(500);
}

void setup()
{
  setupSensors();

  // Sensor failure pins 
  digitalWrite(SENSOR_1_FAILURE_PIN, HIGH);
  digitalWrite(SENSOR_2_FAILURE_PIN, HIGH);
  pinMode(SENSOR_1_FAILURE_PIN, OUTPUT);
  pinMode(SENSOR_2_FAILURE_PIN, OUTPUT);
  digitalWrite(SENSOR_1_FAILURE_PIN, HIGH);
  digitalWrite(SENSOR_2_FAILURE_PIN, HIGH);

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
    upperSensorDistanceAccumulation += (sensor1.readRangeSingleMillimeters());
    lowerSensorDistanceAccumulation += (sensor2.readRangeSingleMillimeters());
    countLoops++;
  }
  else {
    long upperSensorDistance = upperSensorDistanceAccumulation / countLoops;
    long lowerSensorDistance = lowerSensorDistanceAccumulation / countLoops;
    checkAddresses();
  
    // Sensor 1
    if (!sensor1.timeoutOccurred())
    {
      Serial.print("Upper Sensor Distance (mm): ");
      Serial.println(upperSensorDistance);
    } else {
      Serial.println("TIMEOUT 1");
      digitalWrite(SENSOR_1_FAILURE_PIN, LOW);
    }
  
    // Sensor 2
    if (!sensor2.timeoutOccurred())
    {
      Serial.print("Lower Sensor Distance (mm): ");
      Serial.println(lowerSensorDistance);
    } else {
      Serial.println("TIMEOUT 2");
      digitalWrite(SENSOR_2_FAILURE_PIN, LOW);
    }

    Serial.print("UPPER_LIMIT_SWITCH_PIN: ");   
    Serial.print(digitalRead(UPPER_LIMIT_SWITCH_PIN));
    Serial.print(" LOWER_LIMIT_SWITCH_PIN: ");   
    Serial.println(digitalRead(LOWER_LIMIT_SWITCH_PIN));
    upperSensorDistanceAccumulation = 0;
    lowerSensorDistanceAccumulation = 0;
    countLoops = 0;
    lastTime = millis();
  }
}
