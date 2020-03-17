#include <Wire.h>
#include <VL53L0X.h>

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
const float yMax = 315.0;
const float yMin = 234.0;
bool firstLoop = true;

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
    analogWrite(PWM_PIN, 255);
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
      Serial.print("Platform Sensor Distance (mm): ");
      Serial.print(platformSensorDistance);
    } else {
      Serial.println("TIMEOUT 1");
      digitalWrite(PLATFORM_SENSOR_FAILURE_PIN, LOW);
    }
  
    // Sensor 2
    if (!baseSensor.timeoutOccurred())
    {
      Serial.print(" Base Sensor Distance (mm): ");
      Serial.print(baseSensorDistance);
    } else {
      Serial.println("TIMEOUT 2");
      digitalWrite(BASE_SENSOR_FAILURE_PIN, LOW);
    }
//
    Serial.print(" UPPER_LIMIT: ");   
    Serial.print(digitalRead(UPPER_LIMIT_SWITCH_PIN));
    Serial.print(" LOWER_LIMIT: ");   
    Serial.println(digitalRead(LOWER_LIMIT_SWITCH_PIN));
    bool upperLimitSwitch = digitalRead(UPPER_LIMIT_SWITCH_PIN);
    bool lowerLimitSwitch =  digitalRead(LOWER_LIMIT_SWITCH_PIN);
    
    if (baseSensorDistance < 5000){
//      float yDesired = (yMin - yMax)*(81.4*exp(-0.141*baseSensorDistance) - 2)/9 + yMax;
      float yDesiredCalc = (yMin - yMax)*(2068*exp(-0.0141*baseSensorDistance))/228.6 + yMax;
      float yDesired = (yDesiredCalc > yMin) ? yDesiredCalc: yMin;
      Serial.print(" Y Desired: ");
      Serial.println(yDesired);
//      Serial.println("");
      if ((firstLoop == true || (digitalRead(UPPER_LIMIT_SWITCH_PIN) == 0 && digitalRead(LOWER_LIMIT_SWITCH_PIN)== 0) )&& abs(platformSensorDistance - yDesired) > 1){
        if (platformSensorDistance > yDesired){
          Serial.println("Lowering");
          digitalWrite(DIRECTION_UP_PIN, LOW);
          analogWrite(PWM_PIN, 100);
        }
        else if (platformSensorDistance < yDesired){
          Serial.println("Raising");
          digitalWrite(DIRECTION_UP_PIN, HIGH);
          analogWrite(PWM_PIN, 100);
        }
        if (firstLoop == true && digitalRead(UPPER_LIMIT_SWITCH_PIN) == 0 ){
          firstLoop = false;
        }
        delay (100);
        analogWrite(PWM_PIN, 0);
      }
//      else{
//          analogWrite(PWM_PIN, 0);
//      }
      
    }
    else{
      while (digitalRead(UPPER_LIMIT_SWITCH_PIN) == LOW) {
        digitalWrite(DIRECTION_UP_PIN, HIGH);
        analogWrite(PWM_PIN, 100);
      }
      analogWrite(PWM_PIN, 0);
    }
    platformSensorDistanceAccumulation = 0;
    baseSensorDistanceAccumulation = 0;
    countLoops = 0;
    lastTime = millis();
  }
}
