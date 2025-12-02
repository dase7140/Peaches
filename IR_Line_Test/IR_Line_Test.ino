int yellowPin = 33;  // Yellow IR sensor
int whitePin = 32;   // White IR sensor

void setup() {
  pinMode(yellowPin, INPUT);
  pinMode(whitePin, INPUT);
  Serial.begin(9600);
  Serial.println("IR Line Sensor Test - Yellow: Pin 33, White: Pin 32");
  Serial.println("---------------------------------------------------");
}

void loop() {
  int yellowState = digitalRead(yellowPin);
  int whiteState = digitalRead(whitePin);
  
  Serial.print("Yellow: ");
  Serial.print(yellowState);
  Serial.print(" | White: ");
  Serial.println(whiteState);
  
  delay(200);
}