// ##################################
// ##### Inital Setup ###############
// ##################################

// =============================================================
// Ultrasonic Sensor Setup

struct UltrasonicSensor {
  int echoPin;
  int trigPin;
  const char* name;
  bool blocked; 

  
  float Distance(int trigPin, int echoPin) {

    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);

    float duration = pulseIn(echoPin, HIGH, 30000); // Timeout 30ms
    float distance = duration * 0.034 / 2; // Convert to cm

    return distance;
  }
};

// List of sensors
UltrasonicSensor sensors[] = {
  {28, 29, "L", false},
  {30, 31, "R", false},
  {32, 33, "FL", false},
  {34, 35, "FR", false}
};

#define numUSSensors 4

void UltrasonicSensorSetup() {
  // Ultrasonic Sensors
  for (int i = 0; i < numUSSensors; i++) {
    pinMode(sensors[i].trigPin, OUTPUT);
    pinMode(sensors[i].echoPin, INPUT);
  }
}

// NO MEAN RETURN 
void ReadAllUSDistances(float distances[]) {
  for (int i = 0; i < numUSSensors; i++) {
    distances[i] = sensors[i].Distance(sensors[i].trigPin, sensors[i].echoPin);
  }
}

// =============================================================
// Multiplexer and IR Sensor Setup - Adafruit VL53L0X - HiLetGo TCA9548A

#include <Wire.h>
#include <Adafruit_VL53L0X.h>

Adafruit_VL53L0X lox[5];  // for up to 5 sensors

// Front Left - 2
// Front Right - 1
// Left - 0
// Right - 3  
// Back - 4

#define TCAADDR 0x70
#define numIRSensors 5

void tcaSelect(uint8_t channel) {
  if (channel > 7) return;
    Wire.beginTransmission(TCAADDR);
    Wire.write(1 << channel);
    Wire.endTransmission();
    delay(10);
}

void IRSensorSetup() {
    Wire.begin();
    Serial.println("Initializing multiple VL53L0X sensors...");

    for (int i = 0; i < numIRSensors; i++) {
      tcaSelect(i);
        if (!lox[i].begin()) {
          Serial.print("Failed to boot VL53L0X on channel ");
          Serial.println(i);
        }
      Serial.print("VL53L0X sensor initialized on channel ");
      Serial.println(i);
    }
}

void ReadAllIRDistances(int distances[]) {
  for (int i = 0; i < numIRSensors; i++) {
    tcaSelect(i);

    VL53L0X_RangingMeasurementData_t measure;
    lox[i].rangingTest(&measure, false);
    distances[i] = measure.RangeMilliMeter;
  }
}


// =============================================================
// Drive Motor Setup - SparkFun TB6612FNG

#include <SparkFun_TB6612.h>    // Sparkfun Motor Driver TB6612 Library

// Drive Motor Driver Pins
#define PWMA 5
#define AIN2 43
#define AIN1 45
#define STBY 47
#define BIN1 49
#define BIN2 51
#define PWMB 4

const int offsetA = 1;
const int offsetB = -1;

int speed = 255;

Motor left_motor = Motor(AIN1, AIN2, PWMA, offsetA, STBY);
Motor right_motor = Motor(BIN1, BIN2, PWMB, offsetB, STBY); 

void DriveMotorSetup(){
    brake(left_motor, right_motor); // Ensure motors are stopped at startup
}

// ============================================================
// Brush Motor Setup - Pololu DRV8263H

// Brush Motor Setup 
// Brush Motor Driver Pins
#define brush_IN1 7
#define brush_IN2 6

void BrushMotorSetup(){
  pinMode(brush_IN1, OUTPUT);
  pinMode(brush_IN2, OUTPUT);
}

// ============================================================
// Tray Servo Setup - 20G Hobby Servo

// Tray Servo Setup
#include <Servo.h>   // Basic Arduino Servo Library

// Servo Pins
#define tray_servo_pin 9

Servo tray_servo; 

int start_pos = 90;    // Initial position
int down = 90;   // Lowered position
int up = 165;       // Raised position

void TrayServoSetup(){
  tray_servo.attach(tray_servo_pin);
  tray_servo.write(start_pos); // Set to initial position
  delay(500);
}

void LowerTray(){
  tray_servo.write(down);
}

void RaiseTray(){
  tray_servo.write(up);
}

// =============================================================
// Communications Setup

void CommsSetup(){
  Serial.begin(115200);
  delay(1500); // Wait for serial to initialize
} 

// ##################################
// ##### Combination Functions ######
// ##################################

void BrushMotorOn(){
  // Lower Tray
  LowerTray();
  delay(500);

   // Activate brush motor
   digitalWrite(brush_IN1, HIGH);
   digitalWrite(brush_IN2, LOW);
}

void BrushMotorOff(){

  // Deactivate brush motor
  digitalWrite(brush_IN1, HIGH);
  digitalWrite(brush_IN2, HIGH);
  delay(1000); 
     
  // Raise Tray
  RaiseTray();
}




// Front Left - 2
// Front Right - 1
// Left - 0
// Right - 3  
// Back - 4
void WallCheck(){
  int wallCheckLimit = 80;
  int IR_distances[numIRSensors];
  ReadAllIRDistances(IR_distances);
  for (int i = 0; i < numIRSensors; i++) {
    if(IR_distances[i] < wallCheckLimit) {
      if (i == 0) { // Left Sensor
        Serial.println("Left Triggered");
        brake(left_motor, right_motor); 
      }
      else if (i == 1) { // Front Right Sensor
        Serial.println("Front Right Triggered");
        brake(left_motor, right_motor);
      }
      else if (i == 2) { // Front Left Sensor
        Serial.println("Front Left Triggered");
        brake(left_motor, right_motor);
      }
      else if (i == 3) { // Right Sensor
        Serial.println("Right Triggered");
        brake(left_motor, right_motor);
      }
      else if (i == 4) { // Back Sensor
        Serial.println("Back Triggered");
        brake(left_motor, right_motor);
      }
    }
  }
}



bool isDrivingBlind = false;

void DriveBlind(){
  int wallCheckLimit = 150;      // Distance to stop at (mm)
  int frontCheckLimit = 350;
  int IR_distances[numIRSensors];
  int drive_speed = 180;
  int turn_speed = 150;
  
  ReadAllIRDistances(IR_distances);

  // Sensor Mapping Reminder:
  // Left - 0, Front Right - 1, Front Left - 2, Right - 3, Back - 4

  // 1. CHECK FRONT (Emergency Stop & Turn)
  if (IR_distances[2] < frontCheckLimit || IR_distances[1] < frontCheckLimit) {
    Serial.println("Front Obstacle! Backing up...");
    
    // Stop
    brake(left_motor, right_motor);
    delay(200);

    // Back up
    back(left_motor, right_motor, turn_speed);
    delay(500); 
    brake(left_motor, right_motor);
    delay(200);

    // Read sensors again to find the open path
    ReadAllIRDistances(IR_distances);
    int leftSpace = IR_distances[0];
    int rightSpace = IR_distances[3];

    if (leftSpace > rightSpace) {
      Serial.println("Turning Left (More Space)");
      // Turn Left in place
      left_motor.drive(-turn_speed);
      right_motor.drive(turn_speed);
      delay(800); // Adjust this delay to change turn angle
    } else {
      Serial.println("Turning Right (More Space)");
      // Turn Right in place
      left_motor.drive(turn_speed);
      right_motor.drive(-turn_speed);
      delay(800); // Adjust this delay to change turn angle
    }
    
    brake(left_motor, right_motor);
    delay(100); // Brief pause before resuming
  }
  // 2. CHECK SIDES (Course Correction)
  else if (IR_distances[0] < wallCheckLimit) {
    // Left wall is too close -> Steer Right
    // (Keep Left motor fast, slow down Right motor)
    left_motor.drive(drive_speed);
    right_motor.drive(drive_speed / 2);
  }
  else if (IR_distances[3] < wallCheckLimit) {
    // Right wall is too close -> Steer Left
    // (Slow down Left motor, keep Right motor fast)
    left_motor.drive(drive_speed / 2);
    right_motor.drive(drive_speed);
  }
  // 3. PATH CLEAR
  else {
    // Drive straight
    forward(left_motor, right_motor, drive_speed);
  }
}

// #############################################################
// ##### MAIN PROGRAM ##########################################
// #############################################################

void setup() {
  // Serial Communications
  CommsSetup();
  // Ultrasonic Sensors
  UltrasonicSensorSetup();
  // IR Sensors
  IRSensorSetup();
  // Drive Motors
  DriveMotorSetup();
  // Brush Motor
  BrushMotorSetup();  
  // Tray Servo
  TrayServoSetup();

  Serial.println("Setup Complete! ready to recieve inputs");
  BrushMotorOff();
}

void loop() {

  if  (isDrivingBlind){
    DriveBlind();
  } else {
    WallCheck();
  }

  if (Serial.available() > 0){
    String msg = Serial.readStringUntil('\n');
    isDrivingBlind = false;

    //Move forward
    if (msg == "MFD") {
      forward(left_motor, right_motor, speed);
    }
    //Move left
    else if (msg == "ML0") { 
      left_motor.drive(speed);
      right_motor.drive(-speed);
    }
    //Move right
    else if (msg == "MR0") {
      left_motor.drive(-speed);
      right_motor.drive(speed);
    }
    //Move backward
    else if (msg == "MB0") {
      back(left_motor, right_motor, speed);
    }
    //Stop moving
    else if (msg == "MF0") {
      brake(left_motor, right_motor);
    }
    // Activate Brush Motor
    else if (msg == "ABM") {
      BrushMotorOn();
    }
    // Deactivate Brush Motor
    else if (msg == "DBM") {
      BrushMotorOff();
    }
    // Drive Blind using only IR Sensors
    else if (msg == "DBI") {
      isDrivingBlind = true;
    }
    // Lower Tray
    else if (msg == "LTY") {
      LowerTray();
    }
    // Raise Tray
    else if (msg == "RTY") {
      RaiseTray();
    }

    // Read US Sensor Distances
    else if (msg == "RUS") {
      float US_distances[numUSSensors];
      ReadAllUSDistances(US_distances);
      Serial.println("US Distances: ");
        for (int i = 0; i < numUSSensors; i++) {
          Serial.print(sensors[i].name);
          Serial.print("=");
          Serial.print(US_distances[i]);
          if (i < numUSSensors - 1){
            Serial.print(", ");
          }
          Serial.println(); 
        }
    }
    // Read IR Sensor Distances
    else if (msg == "RIS") {
      int IR_distances[numIRSensors];
      ReadAllIRDistances(IR_distances);
      Serial.println("IR Distances: ");
        for (int i = 0; i < numIRSensors; i++) {
          Serial.print(i);
          Serial.print("=");
          Serial.println(IR_distances[i]);
          Serial.println(); 
        }
    }
    else {
      Serial.println("Unknown Command");
      isDrivingBlind = false;

    }
  }
}
