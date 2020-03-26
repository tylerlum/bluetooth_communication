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

const int DIRECTION_SWITCH = 10;
const int STOP_SWITCH = 11;

const int QRD_SENSOR = 14;

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
const int MOTOR_PWM_UP = 20;
const int MOTOR_PWM_DOWN = 15;
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
}

void setup(){
  setupSensors();

  // Moving Average
//  platformSensorAvg.begin();
//  baseSensorAvg.begin();
  
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
  pinMode(DIRECTION_SWITCH, INPUT_PULLUP);
  pinMode(STOP_SWITCH, INPUT_PULLUP);

  pinMode(QRD_SENSOR, INPUT);
}

void loop(){
  Serial.println(analogRead(QRD_SENSOR));
  
  while (!digitalRead(STOP_SWITCH)) {  
    if (digitalRead(DIRECTION_SWITCH) && !digitalRead(LOWER_LIMIT_SWITCH_PIN) ){
      Serial.println("Lowering Arm");
      digitalWrite(DIRECTION_UP_PIN, LOW);
      analogWrite(PWM_PIN, MOTOR_PWM_DOWN);
    }
    else if(!digitalRead(DIRECTION_SWITCH) && !digitalRead(UPPER_LIMIT_SWITCH_PIN)){
      Serial.println("Raising Arm");
      digitalWrite(DIRECTION_UP_PIN, HIGH);
      analogWrite(PWM_PIN, MOTOR_PWM_UP);
    }
  }
  analogWrite(PWM_PIN, 0);
}
