#include <Wire.h>
#include <VL53L0X.h>

VL53L0X sensor;
VL53L0X sensor2;

// Pins
const int PWM_PIN = 10;
const int DIRECTION_UP_PIN = 9;

void setup()
{

  pinMode(7, OUTPUT);
  pinMode(8, OUTPUT);
  digitalWrite(7, LOW);
  digitalWrite(8, LOW);

  // Pins
  pinMode(PWM_PIN, OUTPUT);
  pinMode(DIRECTION_UP_PIN, OUTPUT);
  
  delay(500);
  Wire.begin();

  Serial.begin (9600);

  //SENSOR
  pinMode(7, INPUT);
  delay(150);
  Serial.println("00");
  sensor.init(true);
  Serial.println("01");
  delay(100);
  sensor.setAddress((uint8_t)22);
  Serial.println("02");

  //SENSOR 2
  pinMode(8, INPUT);
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

  sensor.setTimeout(500);
  sensor2.setTimeout(500);
}

void loop()
{
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


  //CHECK DISTANCES

  long upperSensorDistance = (sensor.readRangeSingleMillimeters());
  long lowerSensorDistance = (sensor2.readRangeSingleMillimeters());
  
  //FWD OR SENSOR
  if (!sensor.timeoutOccurred())
  {
    Serial.println("_________________________________");
    Serial.print("Upper Sensor Distance (mm): ");
    Serial.println(upperSensorDistance);
    Serial.println("_________________________________");
    Serial.println("");
  } else {
    Serial.println("TIMEOUT");
  }

  //FLT OR SENSOR2
  if (!sensor2.timeoutOccurred())
  {
    Serial.println("_________________________________");
    Serial.print("Lower Sensor Distance (mm): ");
    Serial.println(lowerSensorDistance);
    Serial.println("_________________________________");
    Serial.println("");
  } else {
    Serial.println("TIMEOUT 2");
  }
  
  Serial.println("__________________________________________________________________");
  Serial.println();
  Serial.println();
  Serial.println();
  Serial.println();

  if (upperSensorDistance > 5000 && lowerSensorDistance < 60) {
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
