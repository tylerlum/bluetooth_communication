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
const int UPPER_LIMIT_SWITCH_PIN = 4;

// Timing
unsigned long lastTime = millis();
const int AVERAGING_TIME_MS = 100;
long upperSensorDistanceAccumulation = 0;
long lowerSensorDistanceAccumulation = 0;
int countLoops = 0;
const float yMax = 237.5;
const float yMin = 209.0;


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

  // Motor pins
  pinMode(PWM_PIN, OUTPUT);
  pinMode(DIRECTION_UP_PIN, OUTPUT);
  
  // Sensor failure pins 
  digitalWrite(SENSOR_1_FAILURE_PIN, LOW);
  digitalWrite(SENSOR_2_FAILURE_PIN, LOW);
  pinMode(SENSOR_1_FAILURE_PIN, OUTPUT);
  pinMode(SENSOR_2_FAILURE_PIN, OUTPUT);
  digitalWrite(SENSOR_1_FAILURE_PIN, LOW);
  digitalWrite(SENSOR_2_FAILURE_PIN, LOW);

  // Limit switch pins
  pinMode(UPPER_LIMIT_SWITCH_PIN, INPUT_PULLUP);

  Serial.print("digitalRead(UPPER_LIMIT_SWITCH_PIN) == LOW is: ");
  Serial.println(digitalRead(UPPER_LIMIT_SWITCH_PIN) == LOW);
  while (digitalRead(UPPER_LIMIT_SWITCH_PIN) == LOW) {
    digitalWrite(DIRECTION_UP_PIN, HIGH);
    analogWrite(PWM_PIN, 255);
  }
  analogWrite(PWM_PIN, 0);

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
//      Serial.print ("Found address: ");
//      Serial.print (i, DEC);
//      Serial.print (" (0x");
//      Serial.print (i, HEX);
//      Serial.println (")");
      count++;
      delay (1);  // maybe unneeded?
    } // end of good response
  } // end of for loop
  Serial.println ("Done.");
  Serial.print ("Found ");
  Serial.print (count, DEC);
  Serial.print (" device(s). ");
//  Serial.println("=================================");
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
//    checkAddresses();
  
    // Sensor 1
    if (!sensor1.timeoutOccurred())
    {
//      Serial.println("_________________________________");
//      Serial.print("Upper Sensor Distance (mm): ");
//      Serial.println(upperSensorDistance);
//      Serial.println("_________________________________");
//      Serial.println("");
    } else {
//      Serial.println("TIMEOUT 1");
      digitalWrite(SENSOR_1_FAILURE_PIN, HIGH);
    }
  
    // Sensor 2
    if (!sensor2.timeoutOccurred())
    {
//      Serial.println("_________________________________");
//      Serial.print("Lower Sensor Distance (mm): ");
//      Serial.println(lowerSensorDistance);
//      Serial.println("_________________________________");
//      Serial.println("");
    } else {
//      Serial.println("TIMEOUT 2");
      digitalWrite(SENSOR_2_FAILURE_PIN, HIGH);
    }
//    
//    Serial.println("__________________________________________________________________");
//    Serial.println();
//    Serial.println();
//    Serial.println();
//    Serial.println();

    // Algorithm for lowering
//    if (upperSensorDistance > 5000 && lowerSensorDistance < 35) {
//      Serial.println("Lowering");
//      digitalWrite(DIRECTION_UP_PIN, LOW);
//      analogWrite(PWM_PIN, 255);
//    } else if (upperSensorDistance < 65) {
//      Serial.println("Lowering");
//      digitalWrite(DIRECTION_UP_PIN, LOW);
//      analogWrite(PWM_PIN, 255);
//    } else {
//      analogWrite(PWM_PIN, 0);
//    }
//      Serial.print(" Y Desired: ");
//      Serial.print(yDesired);
      Serial.println(" Lower sensor: ");
      Serial.print(lowerSensorDistance);
      Serial.print(" Upper sensor: ");
      Serial.println(upperSensorDistance);
//      Serial.println("");
//    if (lowerSensorDistance < 5000){
//      float yDesired = (yMin - yMax)*(81.4*exp(-0.141*lowerSensorDistance) - 2)/9 + yMax;
//      Serial.print(" Y Desired: ");
//      Serial.print(yDesired);
//      Serial.println("");
//      while (abs(upperSensorDistance - yDesired) > 1){
//        if (upperSensorDistance > yDesired){
////          Serial.println("Lowering");
//          digitalWrite(DIRECTION_UP_PIN, LOW);
//          analogWrite(PWM_PIN, 255);
//        }
//        else{
////          Serial.println("Raising");
//          digitalWrite(DIRECTION_UP_PIN, HIGH);
//          analogWrite(PWM_PIN, 255);
//        }
//  
//      }
//      analogWrite(PWM_PIN, 0);
//    }
    

    // Reset globals
    upperSensorDistanceAccumulation = 0;
    lowerSensorDistanceAccumulation = 0;
    countLoops = 0;
    lastTime = millis();
  }
}
