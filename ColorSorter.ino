#include <Servo.h>

Servo myservo;  // create servo object to control a servo
// twelve servo objects can be created on most boards

int pos = 100;  // variable to store the servo position

// Color variables
// Rui Santos
// Complete project details at https://randomnerdtutorials.com
// *********

// TCS230 or TCS3200 pins wiring to Arduino
#define S0 3
#define S1 4
#define S2 10
#define S3 11
#define colorSwitch A0
#define sensorOut 9
#define redLED A2
#define greenLED A4

// Stores frequency read by the photodiodes
int redFrequency = 0;
int greenFrequency = 0;
int blueFrequency = 0;
int colorSwitchHighLow = 0;

// Stores the red, green, and blue colors
int redColor = 0;
int greenColor = 0;
int blueColor = 0;

void setup() {
  myservo.attach(6);  // attaches the servo on pin 11 to the servo object

  // Color sensor setup
  pinMode(S0, OUTPUT);
  pinMode(S1, OUTPUT);
  pinMode(S2, OUTPUT);
  pinMode(S3, OUTPUT);
  pinMode(redLED, OUTPUT);
  pinMode(greenLED, OUTPUT);

  // Setting the sensorOut as an input
  pinMode(sensorOut, INPUT);
  pinMode(colorSwitch, INPUT_PULLUP);

  // Setting frequency scaling to 20%
  digitalWrite(S0, HIGH);
  digitalWrite(S1, LOW);

  // Begins serial communication
  Serial.begin(9600);
}

void loop() {
  // 1) Read sensor and compute R, G, B and switch state
  readColors();

  // 2) Act depending on switch state
  Serial.print(colorSwitch);
  if (colorSwitchHighLow == LOW) {
    handleSwitchLowState();
  } else {
    handleSwitchHighState();
  }
}

// -------------------- FUNCTION DEFINITIONS --------------------

// Reads the color sensor and updates global color variables
void readColors() {
  // --- Color Code ---

  // Setting RED (R) filtered photodiodes to be read
  digitalWrite(S2, LOW);
  digitalWrite(S3, LOW);

  // Read switch state
  colorSwitchHighLow = digitalRead(colorSwitch);

  // Reading the output frequency for RED
  redFrequency = pulseIn(sensorOut, LOW);
  redColor = map(redFrequency, 98, 290, 255, 0);
  delay(100);

  // Setting GREEN (G) filtered photodiodes to be read
  digitalWrite(S2, HIGH);
  digitalWrite(S3, HIGH);

  // Reading the output frequency for GREEN
  greenFrequency = pulseIn(sensorOut, LOW);
  greenColor = map(greenFrequency, 250, 360, 255, 0);
  delay(100);

  // (This was printing the pin number before; most likely you want the state)
  Serial.print("Switch is ");
  Serial.println(colorSwitchHighLow);

  // Setting BLUE (B) filtered photodiodes to be read
  digitalWrite(S2, LOW);
  digitalWrite(S3, HIGH);

  // Reading the output frequency for BLUE
  blueFrequency = pulseIn(sensorOut, LOW);
  blueColor = map(blueFrequency, 400, 700, 255, 0);
  if (blueFrequency < 400) blueColor = 0;
  if (blueFrequency > 700) blueColor = 0;
  delay(100);
}

// Behavior when colorSwitchHighLow == LOW
void handleSwitchLowState() {
  analogWrite(greenLED, 140);
  analogWrite(redLED, 0);  // Turn on green LED (same as original)
  if (blueFrequency > 245 && redFrequency > 240 && greenFrequency > 280) {
    Serial.println(" - BLACK detected!");
    myservo.write(100);
  }
  else if (redFrequency < greenFrequency) {
    Serial.println(" - RED detected!");
    // LeftDrop
    for (pos = 100; pos <= 180; pos += 1) {
      myservo.write(pos);
      delay(20);
    }
    for (pos = 180; pos >= 100; pos -= 1) {
      myservo.write(pos);
      delay(20);
    }
  }

  else if (greenFrequency < redFrequency) {
    Serial.println(" - GREEN detected!");
    // RightDrop
    for (pos = 100; pos >= 0; pos -= 1) {
      myservo.write(pos);
      delay(20);
    }
    for (pos = 0; pos <= 100; pos += 1) {
      myservo.write(pos);
      delay(20);
    }
  }


}

// Behavior when colorSwitchHighLow == HIGH
void handleSwitchHighState() {
  analogWrite(redLED, 140);  // Turn on red LED (same as original)
  analogWrite(greenLED,0);
  if (blueFrequency > 245 && redFrequency > 240 && greenFrequency > 280) {
    Serial.println(" - BLACK detected!");
    myservo.write(100);
  }
  else if (redFrequency < greenFrequency) {
    Serial.println(" - RED detected!");
    // LeftDrop (reversed direction vs LOW state)
    for (pos = 100; pos >= 0; pos -= 1) {
      myservo.write(pos);
      delay(20);
    }
    for (pos = 0; pos <= 100; pos += 1) {
      myservo.write(pos);
      delay(20);
    }
  }

  else if (greenFrequency < redFrequency) {
    Serial.println(" - GREEN detected!");
    // RightDrop (reversed direction vs LOW state)
    for (pos = 100; pos <= 180; pos += 1) {
      myservo.write(pos);
      delay(20);
    }
    for (pos = 180; pos >= 100; pos -= 1) {
      myservo.write(pos);
      delay(20);
    }
  }

}