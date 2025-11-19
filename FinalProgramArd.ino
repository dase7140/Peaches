#include <SparkFun_TB6612.h>

struct USS {
  int trig;
  int echo;
  const char* name;
};

USS sensors[] = {
  {27, 26, "FL"},
  {25, 24, "FR"},
  {31, 30, "R"},
  {29, 28, "L"},
  {33, 32, "B"}
};

const int NUM_SENSORS = sizeof(sensors) / sizeof(sensors[0]);

int OBSTACLE_THRESHOLD = 600; 

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



float readSingleUS(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  long duration = pulseIn(echoPin, HIGH, 20000); // 20ms timeout

  if (duration == 0) return 999;   // No echo → treat as far away

  float distance = duration * 0.0343 / 2.0;  
  return distance;
}



// READ ALL SENSORS & SEND FLAGS BACK TO PYTHON
// Output format example:
// FL:0 FR:1 L:0 R:0 B:0\n
//
// 1 = obstacle detected
// 0 = clear

void readTOFSensors() {
  String output = "";

  for (int i = 0; i < NUM_SENSORS; i++) {
    float dist = readSingleUS(sensors[i].trig, sensors[i].echo);
    bool flag = (dist < OBSTACLE_THRESHOLD);

    output += sensors[i].name;
    output += ":";
    output += flag ? "1" : "0";

    if (i < NUM_SENSORS - 1) {
      output += " ";  // only add space BETWEEN entries
    }
  }

  Serial.println(output);  
}



void setup() {
  Serial.begin(115200);

  // Configure pins
  for (int i = 0; i < NUM_SENSORS; i++) {
    pinMode(sensors[i].trig, OUTPUT);
    pinMode(sensors[i].echo, INPUT);
  }
}



void loop() {
readTOFSensors();
  if (Serial.available() > 0) {
    String msg = Serial.readStringUntil('\n');

    if (msg == "PSD") {
      readTOFSensors();    // Python requests sensor 
    }

    else if (msg == "MFD") {
      spinnerMotor(50);
      motor1.brake();
      motor2.brake();
    }

    else if (msg == "ML0") {
      motor1.drive(190);
      motor2.drive(-190);
    }

    else if (msg == "MR0") {
      motor1.drive(-190);
      motor2.drive(190);
    }

    else if (msg == "TNF") {
      motor1.brake();
      motor2.brake();
    }

    else if (msg == "MB0") {
      motor1.drive(-190);
      motor2.drive(-190);
    }

    else if (msg == "MF0") {
      motor1.brake();
      motor2.brake();
    }
  }
}


void spinnerMotor(int motorSpeed) {
  digitalWrite(BAIN1, HIGH);
  digitalWrite(BAIN2, LOW);
  analogWrite(BPWMA, motorSpeed);
  digitalWrite(BSTBY, HIGH);
}
