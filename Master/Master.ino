// include the library code:
#include <LiquidCrystal.h>

// LCD object. Initialize the library with the numbers of the interface pins
LiquidCrystal lcd(12, 11, 5, 4, 3, 2);

// Pins
const int LED_PIN = 10;

// Constants
const int numSamples = 100;

// Global variables
int brightness = 0;
int averageBrightness = 0;
int averageCount = 0;

void setup() {
  // Setup Serial port
  Serial.begin(9600);

  // Setup LED output pin
  pinMode(LED_PIN, OUTPUT);

  // Setup the LCD's number of columns and rows:
  lcd.begin(16, 2);
}

void loop() {
  // Update brightness value, if available
  if (Serial.available() > 0) {
    brightness = Serial.read();
  }

  // Write to LED
  analogWrite(LED_PIN, brightness);

  // Print averageBrightness every numSamples
  if (++averageCount == numSamples) {
    lcd.clear();
    lcd.print(averageBrightness / numSamples);
    averageCount = 0;
    averageBrightness = 0;
  } 

  // Add up brightness samples
  else {
    averageBrightness += brightness;
  }
}
