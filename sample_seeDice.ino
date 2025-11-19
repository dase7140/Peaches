#include <SparkFun_TB6612.h>


const int trigPin = 9;
const int echoPin = 10;

float duration, distance;

//for the drive motors
// Motor Driver 1
#define APWMA 40
#define AAIN1 42
#define AAIN2 44
#define ASTBY 46
#define ABIN2 48
#define ABIN1 50
#define APWMB 52
// Motor Driver 2
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

void setup() {
  // put your setup code here, to run once:
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

 //pinModes for the drive motors
  pinMode(APWMA, OUTPUT);
  pinMode(AAIN1, OUTPUT);
  pinMode(AAIN2, OUTPUT);
  pinMode(APWMB, OUTPUT);
  pinMode(ABIN1, OUTPUT);
  pinMode(ABIN2, OUTPUT);
  pinMode(ASTBY, OUTPUT);

  //pinModes for the spinner motor
  pinMode(BPWMA, OUTPUT);
  pinMode(BAIN1, OUTPUT);
  pinMode(BAIN2, OUTPUT);
  pinMode(BSTBY, OUTPUT);
  
  Serial.begin(115200);
  //Serial.setTimeout(50);
}

void loop() {
  // put your main code here, to run repeatedly:

  if (Serial.available() > 0){
    String msg = Serial.readStringUntil('\n');
   
  if (msg == "MFD") { 
    
    //move forward, turn on brush
       //spinnerMotor(50);
    //for testing: just stay stationary
       motor1.brake();
       motor2.brake();
  }
  else if (msg == "ML0") {
    //Move left

   motor1.drive(190);
   motor2.drive(-190);

    }
  else if (msg == "MR0") {
    //Move right

   motor1.drive(-190);
   motor2.drive(190);

    }
  else if (msg == "TNF") { 
    //No dice found, as of now: Stop

   motor1.brake();
   motor2.brake();

    }
  else if (msg == "MB0") { 

   motor1.drive(-190);
   motor2.drive(-190);
    }
  else if (msg == "MF0") { 
    
    //move forward, turn on brush
    //for testing: just stay stationary
       motor1.brake();
       motor2.brake();
  }

  //Serial.flush();

}
//functions

int readfromUS1(){
  
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);
    digitalWrite(trigPin,HIGH);
    delayMicroseconds(2);
    digitalWrite(trigPin, LOW);

    duration = pulseIn(echoPin, HIGH);
    
}

void spinnerMotor(int motorSpeed){
 
  digitalWrite(BAIN1, HIGH);
  digitalWrite(BAIN2, LOW);  
  analogWrite(BPWMA, motorSpeed);
  digitalWrite(BSTBY, HIGH);
}
