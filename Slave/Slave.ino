#include <Wire.h>
#include <VL53L0X.h>

// Sensors
VL53L0X sensor1;
VL53L0X sensor2;

// Pins
const int PWM_PIN = 10;
const int DIRECTION_UP_PIN = 9;
const int SENSOR_1_XSHUT_PIN = 7;
const int SENSOR_2_XSHUT_PIN = 8;

const int SENSOR_1_FAILURE_PIN = 2;
const int SENSOR_2_FAILURE_PIN = 3;

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
  Serial.println("00");
  sensor1.init(true);
  Serial.println("01");
  delay(100);
  sensor1.setAddress((uint8_t)22);
  Serial.println("02");

  // Sensor 2 setup
  pinMode(SENSOR_2_XSHUT_PIN, INPUT);  // Turn on sensor. Can't set OUTPUT HIGH b/c XSHUT can't handle 5V
  delay(150);
  sensor2.init(true);
  Serial.println("03");
  delay(100);
  sensor2.setAddress((uint8_t)25);
  Serial.println("04");

  Serial.println("");
  Serial.println("addresses set");
  Serial.println("");
  Serial.println("");

  sensor1.setTimeout(500);
  sensor2.setTimeout(500);
}

void setup()
{
  setupSensors();
  
  // Pins
  pinMode(PWM_PIN, OUTPUT);
  pinMode(DIRECTION_UP_PIN, OUTPUT);

  digitalWrite(SENSOR_1_FAILURE_PIN, LOW);
  digitalWrite(SENSOR_2_FAILURE_PIN, LOW);
  pinMode(SENSOR_1_FAILURE_PIN, OUTPUT);
  pinMode(SENSOR_2_FAILURE_PIN, OUTPUT);
  digitalWrite(SENSOR_1_FAILURE_PIN, LOW);
  digitalWrite(SENSOR_2_FAILURE_PIN, LOW);
}

void checkAddresses() {
  Serial.println("__________________________________________________________________");
  Serial.println("");
  Serial.println("=================================");
  Serial.println ("I2C scanner. Scanning ...");
  byte count = 0;


  for (byte i = 1; i < 30; i++)
  {

    Wire.beginTransmission (i);
    if (Wire.endTransmission () == 0)
    {
      Serial.print ("Found address: ");
      Serial.print (i, DEC);
      Serial.print (" (0x");
      Serial.print (i, HEX);
      Serial.println (")");
      count++;
      delay (1);  // maybe unneeded?
    } // end of good response
  } // end of for loop
  Serial.println ("Done.");
  Serial.print ("Found ");
  Serial.print (count, DEC);
  Serial.println (" device(s).");
  Serial.println("=================================");
}

void loop()
{
  checkAddresses();

  //CHECK DISTANCES
  long upperSensorDistance = (sensor1.readRangeSingleMillimeters());
  long lowerSensorDistance = (sensor2.readRangeSingleMillimeters());
  
  // Sensor 1
  if (!sensor1.timeoutOccurred())
  {
    Serial.println("_________________________________");
    Serial.print("Upper Sensor Distance (mm): ");
    Serial.println(upperSensorDistance);
    Serial.println("_________________________________");
    Serial.println("");
  } else {
    Serial.println("TIMEOUT 1");
    digitalWrite(SENSOR_1_FAILURE_PIN, HIGH);
  }

  // Sensor 2
  if (!sensor2.timeoutOccurred())
  {
    Serial.println("_________________________________");
    Serial.print("Lower Sensor Distance (mm): ");
    Serial.println(lowerSensorDistance);
    Serial.println("_________________________________");
    Serial.println("");
  } else {
    Serial.println("TIMEOUT 2");
    digitalWrite(SENSOR_2_FAILURE_PIN, HIGH);
  }
  
  Serial.println("__________________________________________________________________");
  Serial.println();
  Serial.println();
  Serial.println();
  Serial.println();
  
  // Algorithm for lowering
  if (lowerSensorDistance < 60) {
    Serial.println("Lowering");
    digitalWrite(DIRECTION_UP_PIN, LOW);
    analogWrite(PWM_PIN, 255);
  } else if (upperSensorDistance < 60) {
    Serial.println("Lowering");
    digitalWrite(DIRECTION_UP_PIN, LOW);
    analogWrite(PWM_PIN, 255);
  } else {
    analogWrite(PWM_PIN, 0);
  }

  delay(500);//can change to a lower time like 100
}
