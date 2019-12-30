// Pins
const int LED_PIN = 10;

// Global variables
int brightness = 1020;

void setup() {
  // Setup Serial port
  Serial.begin(9600);

  // Setup LED output pin
  pinMode(LED_PIN, OUTPUT);
}

void loop() {
  // Update brightness value, if available
  if (Serial.available() > 0) { brightness = Serial.read(); } 

  // Write to LED
  analogWrite(LED_PIN, brightness);
}
