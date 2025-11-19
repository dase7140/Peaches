#include <SparkFun_TB6612.h>

struct UltrasonicSensor {
  int echoPin;
  int trigPin;
  const char* name;
};

// List of sensors
UltrasonicSensor sensors[] = {
  {27, 26, "FL"},
  {29, 28, "FR"},
  {31, 30, "R"},
  {25, 24, "L"},
  {23, 22, "B"}
};

const int numSensors = sizeof(sensors) / sizeof(sensors[0]);

void setup() {
  Serial.begin(115200);

  // Initialize pins
  for (int i = 0; i < numSensors; i++) {
    pinMode(sensors[i].trigPin, OUTPUT);
    pinMode(sensors[i].echoPin, INPUT);
  }
}

long readUltrasonic(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  long duration = pulseIn(echoPin, HIGH, 30000); // Timeout 30ms
  long distance = duration * 0.034 / 2; // Convert to cm
  return distance;
}

#define APWMA 40
#define AAIN1 42
#define AAIN2 44
#define ASTBY 46
#define ABIN2 48
#define ABIN1 50
#define APWMB 52

#define BPWMA 41
#define BAIN1 43
#define BAIN2 45
#define BSTBY 47
#define BBIN2 49
#define BBIN1 51
#define BPWMB 53

const int offsetA = 1;
const int offsetB = 1;
const int offsetC = 1;

Motor motor1 = Motor(AAIN1, AAIN2, APWMA, offsetA, ASTBY);
Motor motor2 = Motor(ABIN1, ABIN2, APWMB, offsetB, ASTBY);
Motor motor3 = Motor(BAIN1, BAIN2, BPWMA, offsetC, BSTBY);


// READ ALL SENSORS & SEND FLAGS BACK TO PYTHON
// Output format example:
// FL:0 FR:1 L:0 R:0 B:0\n
//
// 1 = obstacle detected
// 0 = clear

  

void loop() {

  for (int i = 0; i < numSensors; i++) {
    long distance = readUltrasonic(sensors[i].trigPin, sensors[i].echoPin);
    Serial.print(sensors[i].name);
    Serial.print(": ");
    if (distance == 0) {
      Serial.println("Out of range");
    } else {
      Serial.print(distance);
      Serial.println(" cm");
    }
  }
  Serial.println("----------------");
  delay(500); // Read every 0.5 seconds
}
