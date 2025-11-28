#include <SparkFun_TB6612.h>

struct UltrasonicSensor {
  int echoPin;
  int trigPin;
  const char* name;
};

// List of sensors
UltrasonicSensor sensors[] = {
  {35, 34, "FL"},
  {33, 32, "FR"},
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

const int threshold = 20;

  
void loop() {
 if (Serial.available()) {
    String request = Serial.readStringUntil('\n');
    request.trim();

  if (request == "PSD") {
      

    String output = "";

    for (int i = 0; i < numSensors; i++) {

      long distance = readUltrasonic(sensors[i].trigPin, sensors[i].echoPin);

      int blocked = 0;

    // Ignore invalid readings (0 = timeout/no echo)
      if (distance > 0 && distance <= threshold) {
        blocked = 1;
    }

    output += String(sensors[i].name) + ":" + String(blocked);
    if (i < numSensors - 1) output += " ";
  }

  Serial.println(output);  // Send result to Python
  delay(50);
}
}
}
