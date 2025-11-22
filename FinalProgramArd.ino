#include <SparkFun_TB6612.h>

struct UltrasonicSensor {
  int echoPin;
  int trigPin;
  const char* name;
  bool blocked; 
};

// List of sensors
UltrasonicSensor sensors[] = {
  {35, 34, "FL", false},
  {33, 32, "FR", false},
  {31, 30, "R",  false},
  {25, 24, "L",  false},
  {23, 22, "B",  false}
};

const int numSensors = sizeof(sensors) / sizeof(sensors[0]);

//for the drive motors
// Motor Driver 1
// Motor Driver 2
#define BPWMA 41
#define BAIN1 43
#define BAIN2 45
#define BSTBY 47
#define BBIN1 49
#define BBIN2 51
#define BPWMB 53

const int offsetA = 1;
const int offsetB = 1;
const int offsetC = 1;

Motor motor1 = Motor(BAIN1, BAIN2, BPWMA, offsetC, BSTBY);
Motor motor2 = Motor(BBIN1, BBIN2, BPWMB, offsetC, BSTBY); //right

void spinnerMotor(int motorSpeed){
 
  digitalWrite(BAIN1, HIGH);
  digitalWrite(BAIN2, LOW);  
  analogWrite(BPWMA, motorSpeed);
  digitalWrite(BSTBY, HIGH);
}
const int threshold = 20;


void setup() {
  // put your setup code here, to run once:

 //pinModes for the drive motors
  
  //pinModes for the spinner motor
  pinMode(BPWMA, OUTPUT);
  pinMode(BAIN1, OUTPUT);
  pinMode(BAIN2, OUTPUT);
  pinMode(BSTBY, OUTPUT);
  pinMode(BPWMB, OUTPUT);
  pinMode(BBIN1, OUTPUT);
  pinMode(BBIN2, OUTPUT);
  
  Serial.begin(115200);
  //Serial.setTimeout(50);

  //Sensors:
    for (int i = 0; i < numSensors; i++) {
    pinMode(sensors[i].trigPin, OUTPUT);
    pinMode(sensors[i].echoPin, INPUT);
  }


}

//Function for ultrasonics
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
int speed = 150; //solution to make it slower: spike PWM then go slower

const int thresholdBlocked = 20;  // when definitely blocked
const int thresholdClear   = 25;

long readUltrasonicMedian(int trigPin, int echoPin) {
  long readings[3];
  for (int i = 0; i < 3; i++) {
    readings[i] = readUltrasonic(trigPin, echoPin);
    delay(5);
  }
  // simple sort to get median
  for (int i = 0; i < 2; i++)
    for (int j = i+1; j < 3; j++)
      if (readings[j] < readings[i]) {
        long tmp = readings[i];
        readings[i] = readings[j];
        readings[j] = tmp;
      }
  return readings[1]; // median
}

void loop() {
  // put your main code here, to run repeatedly:

  if (Serial.available() > 0){
  String msg = Serial.readStringUntil('\n');
   
  if (msg == "MFD") { 
    
    //move forward, turn on brush
       motor1.drive(speed);
       motor2.drive(speed);
  }
  else if (msg == "ML0") {
    //Move left

   motor1.drive(speed);
   motor2.drive(-speed);

    }
  else if (msg == "MR0") {
    //Move right

   motor1.drive(-speed);
   motor2.drive(speed);

    }
  else if (msg == "TNF") { 
    //No dice found, as of now: Stop
    
   motor1.drive(speed);
   motor2.drive(speed);

    }
  else if (msg == "MB0") { 

   motor1.drive(-speed);
   motor2.drive(-speed);
    }
  else if (msg == "MF0") { 
    
    //for testing: just stay stationary
       motor1.drive(speed);
       motor2.drive(speed);
  }
  else if (msg == "PSD") {
    String output = "";

      for (int i = 0; i < numSensors; i++) {
        long distance = readUltrasonicMedian(sensors[i].trigPin, sensors[i].echoPin);
        bool blocked = sensors[i].blocked;

        // Hysteresis logic
        if (distance > 0 && distance <= thresholdBlocked) {
          blocked = true;
        } else if (distance > thresholdClear) {
          blocked = false;
        }
        sensors[i].blocked = blocked; // update stored state

        output += String(sensors[i].name) + ":" + (blocked ? "1" : "0");
        if (i < numSensors - 1) output += " ";
      }

      Serial.println(output); // send dictionary string
      delay(50);
    }


}
}
