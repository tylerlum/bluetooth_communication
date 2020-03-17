#include <Wire.h>
#include <VL53L0X.h>
#include <movingAvg.h>

// Sensors
VL53L0X platformSensor;
VL53L0X baseSensor;

// Pins
const int PWM_PIN = 9;
const int DIRECTION_UP_PIN = 8;
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

//Averaging
movingAvg platformSensorAvg(10);
movingAvg baseSensorAvg(10);

// System Constants
const float Y_MAX = 315.0;
const float Y_MIN = 234.0;
const int MOTOR_PWM = 50;
bool firstLoop = true;

float platformSensorDistance, baseSensorDistance;
float desiredHeight;
int prevHeightError = 0;
int heightError;
bool outlierPlat = false;
bool outlierBase = false;

// PID Variables
float gain, p, i, d;
float K = 0.25;
int Kp = 1;
int Ki = 0;
int Kd = 1;
int iMax = 0;

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

  // Platform sensor setup
  pinMode(PLATFORM_SENSOR_XSHUT_PIN, INPUT);  // Turn on sensor. Can't set OUTPUT HIGH b/c XSHUT can't handle 5V
  delay(150);
  platformSensor.init(true);
  delay(100);
  platformSensor.setAddress((uint8_t)22);

  // Base sensor setup
  pinMode(BASE_SENSOR_XSHUT_PIN, INPUT);  // Turn on sensor. Can't set OUTPUT HIGH b/c XSHUT can't handle 5V
  delay(150);
  baseSensor.init(true);
  delay(100);
  baseSensor.setAddress((uint8_t)25);

  platformSensor.setTimeout(500);
  baseSensor.setTimeout(500);

}

void setup(){
  setupSensors();

  // Moving Average
  platformSensorAvg.begin();
  baseSensorAvg.begin();
  
  // Motor pins
  pinMode(PWM_PIN, OUTPUT);
  pinMode(DIRECTION_UP_PIN, OUTPUT);
  
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

  while (digitalRead(UPPER_LIMIT_SWITCH_PIN) == LOW) {
    digitalWrite(DIRECTION_UP_PIN, HIGH);
    analogWrite(PWM_PIN, 100);
  }
  analogWrite(PWM_PIN, 0);
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
  int start = millis();
  int readingPlat = platformSensor.readRangeSingleMillimeters();
  int readingBase = baseSensor.readRangeSingleMillimeters();

  if (readingPlat > 5000 && outlierPlat == false){
    outlierPlat = true;
    platformSensorDistance = platformSensorAvg.getAvg();
  }
  else {
    platformSensorDistance = platformSensorAvg.reading(readingPlat);
    outlierPlat = false;
  }
  if (readingBase > 5000 && outlierBase == false){
    outlierBase = true;
    baseSensorDistance = baseSensorAvg.getAvg();
  }
  else {
    baseSensorDistance = baseSensorAvg.reading(readingBase);
    outlierBase = false;
  }
 
//  baseSensorDistance = baseSensorAvg.reading(baseSensor.readRangeSingleMillimeters());
  // Sensor 1
  if (!platformSensor.timeoutOccurred())
  {
//    Serial.print("PlatformSensor(mm): ");
    Serial.print(platformSensorDistance);
    Serial.print(", ");
  } else {
    Serial.println("TIMEOUT 1");
    digitalWrite(PLATFORM_SENSOR_FAILURE_PIN, LOW);
  }

  // Sensor 2
  if (!baseSensor.timeoutOccurred())
  {
//    Serial.print(" BaseSensor(mm): ");
    Serial.print(baseSensorDistance);
    Serial.print(", ");
  } else {
    Serial.println("TIMEOUT 2");
    digitalWrite(BASE_SENSOR_FAILURE_PIN, LOW);
  }
//
//  Serial.print(" UPPER_LIMIT: ");   
//  Serial.print(digitalRead(UPPER_LIMIT_SWITCH_PIN));
//  Serial.print(" LOWER_LIMIT: ");   
//  Serial.println(digitalRead(LOWER_LIMIT_SWITCH_PIN));
  
  if (baseSensorDistance < 5000){
    float desiredHeightCalc = (Y_MIN - Y_MAX)*(2068*exp(-0.0141*baseSensorDistance))/228.6 + Y_MAX;
    if (desiredHeightCalc < Y_MIN){
      desiredHeight = Y_MIN;
    }
    else if (desiredHeightCalc > Y_MAX){
      desiredHeight = Y_MAX;
    }
    else {
      desiredHeight = desiredHeightCalc;
    }

    prevHeightError = heightError;
    heightError = desiredHeight - platformSensorDistance;
//    Serial.print("Desired Height: ");
    Serial.print(desiredHeight);
    Serial.print(", ");
//    Serial.print(" Height Error: ");
    Serial.print(heightError);
    Serial.print(", ");
    if (abs(heightError) > 2){
      p = Kp*heightError;
      d = Kd*(heightError - prevHeightError);
      i = Ki*heightError + i;
        if (abs(i) > iMax){
          i = (i/abs(i))*iMax;
        }
      gain = K*(p + i + d);
//      Serial.print(" Gain: ");
      Serial.print(gain);
      Serial.print(", ");
      if (gain > 0 && !digitalRead(UPPER_LIMIT_SWITCH_PIN)){
//        Serial.println("Raising");
        digitalWrite(DIRECTION_UP_PIN, HIGH);
        int pwm = MOTOR_PWM < 255? MOTOR_PWM + abs(gain): 255;
        analogWrite(PWM_PIN, pwm);
      }
      else if(gain < 0 && !digitalRead(LOWER_LIMIT_SWITCH_PIN)){
//        Serial.println("Lowering");
        digitalWrite(DIRECTION_UP_PIN, LOW);
        int pwm = MOTOR_PWM < 255? MOTOR_PWM + abs(gain): 255;
        analogWrite(PWM_PIN, pwm);
      }
    }
    else {
      analogWrite(PWM_PIN, 0);
    }
  }
  else{
//    Serial.println("Reseting Height");
    while (digitalRead(UPPER_LIMIT_SWITCH_PIN) == LOW) {
      digitalWrite(DIRECTION_UP_PIN, HIGH);
      analogWrite(PWM_PIN, 100);
    }
    analogWrite(PWM_PIN, 0);
  }

  int time1 = millis() - start;
//  Serial.print("Loop Time: ");
  Serial.println(time1);
}
