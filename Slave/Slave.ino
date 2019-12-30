// Pins
const int POT_PIN = A0;
const int LED_PIN = 10;

// Global variables
int potValue = 0;

void setup() {
  // Setup Serial port
  Serial.begin(9600);
}

void loop() {
  // Read potentiometer value
  potValue = analogRead(POT_PIN);

  // Write to Serial port and LED
  Serial.write(potValue);
  analogWrite(LED_PIN, potValue);

  // Throttle loop
  delay(10);
}
