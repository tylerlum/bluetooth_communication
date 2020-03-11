// Pins
const int SET_DIRECTION_PIN = 10;
const int PWM_PIN = 9;
const int DIRECTION_1_PIN = 0;
const int DIRECTION_2_PIN = 1;
const int ON_PIN = 2;

// Max motor speed
const int MOTOR_SPEED = 255;  // [0,255]

// Global variables for readings, start inactive
int isDirection1 = HIGH;
int isDirection2 = HIGH;
int isOn = HIGH;

void setup() {
  // Setup Serial port
  Serial.begin(9600);

  // Setup pins
  pinMode(PWM_PIN, OUTPUT);
  pinMode(SET_DIRECTION_PIN, OUTPUT);
  pinMode(DIRECTION_1_PIN, INPUT_PULLUP);  // Active Low
  pinMode(DIRECTION_2_PIN, INPUT_PULLUP);  // Active Low
  pinMode(ON_PIN, INPUT_PULLUP);           // Active Low
}

void loop() {
  // Read buttons
  isOn = digitalRead(ON_PIN); 
  isDirection1 = digitalRead(DIRECTION_1_PIN);
  isDirection2 = digitalRead(DIRECTION_2_PIN);

  // isOn case
  if (isOn == LOW) {
    // Set direction1
    if (isDirection1 == LOW) {
      Serial.println("Direction 1");
      digitalWrite(SET_DIRECTION_PIN, HIGH);
      analogWrite(PWM_PIN, MOTOR_SPEED);
    } 

    // Set direction2
    else if (isDirection2 == LOW) {
      Serial.println("Direction 2");
      digitalWrite(SET_DIRECTION_PIN, LOW);
      analogWrite(PWM_PIN, MOTOR_SPEED);
    }

    else {
      Serial.println("No direction set. Bringing down motor speed.");
      analogWrite(PWM_PIN, 0);
    }
  }

  // Not isOn case
  else {
    Serial.println("Not on");
  }

  Serial.println("*******************");

}
