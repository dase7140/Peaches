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
    //delay(10);
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

// Speed levels for each direction
const int SPEED_1 = 150;
const int SPEED_2 = 180;
const int SPEED_3 = 200;
const int SPEED_4 = 220;
const int SPEED_5 = 255;

int current_speed = SPEED_2; // Default speed

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

void turnLeft(int spd){
  left_motor.drive(-spd);
  right_motor.drive(spd);
}

void turnRight(int spd){
  left_motor.drive(spd);
  right_motor.drive(-spd);
}


// Front Left - 1
// Front Right - 2
// Left - 0
// Right - 3  
// Back - 4
void WallCheck(){
  int wallCheckLimit = 200;
  int IR_distances[numIRSensors];
  ReadAllIRDistances(IR_distances);
  for (int i = 0; i < numIRSensors; i++) {
    if(IR_distances[i] < wallCheckLimit) {
      if (i == 0) { // Left Sensor
        Serial.println("L");
        brake(left_motor, right_motor); 
      }
      else if (i == 1) { // Front Left Sensor
        Serial.println("FL");
        brake(left_motor, right_motor);
      }
      else if (i == 2) { // Front Right Sensor
        Serial.println("FR");
        brake(left_motor, right_motor);
      }
      else if (i == 3) { // Right Sensor
        Serial.println("R");
        brake(left_motor, right_motor);
      }
      else if (i == 4) { // Back Sensor
        Serial.println("B");
        brake(left_motor, right_motor);
      }
    }
  }
}



bool isDrivingBlind = false;

void DriveBlind(){
  int wallCheckLimit = 80;      // Distance to stop at (mm)
  int frontCheckLimit = 300;
  int IR_distances[numIRSensors];
  int drive_speed = SPEED_2;
  int turn_speed = SPEED_2;
  
  ReadAllIRDistances(IR_distances);

  // Sensor Mapping Reminder:
  // Left - 0, Front Left - 1, Front Right - 2, Right - 3, Back - 4

  // 1. CHECK FRONT (Emergency Stop & Turn)
  if (IR_distances[2] < frontCheckLimit || IR_distances[1] < frontCheckLimit) {
    // Stop
    brake(left_motor, right_motor);
    delay(200);
    back(left_motor, right_motor, turn_speed);
    delay(100); 
    brake(left_motor, right_motor);
    delay(200);
      
    // Read sensors again to find the open path
    ReadAllIRDistances(IR_distances);
    int leftSpace = (IR_distances[0] + IR_distances[1]) / 2;  // Average of Left and Front Left
    int rightSpace = (IR_distances[2] + IR_distances[3]) / 2; // Average of Front Right and Right

    if (leftSpace > rightSpace) {
      // Turn Left in place
      turnLeft(turn_speed);
      delay(500); // Adjust this delay to change turn angle
    } else {
      // Turn Right in place
      turnRight(turn_speed);
      delay(500); // Adjust this delay to change turn angle
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

// State Machine for Robot Control
enum RobotState {
  IDLE,
  MOVING_FORWARD,
  TURNING_LEFT,
  TURNING_RIGHT,
  MOVING_BACKWARD,
  DRIVING_BLIND,
  SEARCHING_YELLOW
};

RobotState currentState = IDLE;
unsigned long stateStartTime = 0;
int yellow_search_step = 0;

void YellowSearch(){

  //checks current search step to determine direction; backs up for 3, turns right for 5, turns left for 10, 
  // right for 15, left for 20, etc. After 60 steps, timeout
  if(0 <= yellow_search_step && yellow_search_step <= 2){
    int IR_distances[5];
    ReadAllIRDistances(IR_distances);
    if(IR_distances[4] > 200){
      back(left_motor, right_motor, current_speed);
    } else {
      // Can't back up, skip to turning
      yellow_search_step = 3;
    }
  }
  else if(3 <= yellow_search_step && yellow_search_step <= 7){
    turnRight(current_speed);
  }
  else if(8 <= yellow_search_step && yellow_search_step <= 17){
    turnLeft(current_speed);
  }
  else if(18 <= yellow_search_step && yellow_search_step <= 32){
    turnRight(current_speed);
  }
  else if(33 <= yellow_search_step && yellow_search_step <= 52){
    turnLeft(current_speed);
  }
  else {
    turnRight(current_speed);
  }
  yellow_search_step += 1;
  delay(100);
}

// #############################################################
// ##### MAIN PROGRAM ##########################################
// #############################################################

void setup() {
  // Serial Communications
  CommsSetup();
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
  // Process serial commands FIRST before executing states
  if (Serial.available() > 0){
    String msg = Serial.readStringUntil('\n');
    msg.trim(); // Remove any leading/trailing whitespace

    // Ignore empty messages
    if (msg.length() == 0){
      return; 
    }
    
    // Process commands and change state
    // Search for Yellow Line
    if (msg == "YLL") {
      yellow_search_step = 0;
      currentState = SEARCHING_YELLOW;
      stateStartTime = millis();
      Serial.println("ACK:YLL");
    }
    // Move Forward - Speed Levels 1-5 
    else if (msg == "MF1") {
      currentState = MOVING_FORWARD;
      current_speed = SPEED_1;
      forward(left_motor, right_motor, current_speed);
      Serial.println("ACK:" + msg);
    }
    else if (msg == "MF2") {
      currentState = MOVING_FORWARD;
      current_speed = SPEED_2;
      forward(left_motor, right_motor, current_speed);
      Serial.println("ACK:MF2");
    }
    else if (msg == "MF3") {
      currentState = MOVING_FORWARD;
      current_speed = SPEED_3;
      forward(left_motor, right_motor, current_speed);
      Serial.println("ACK:MF3");
    }
    else if (msg == "MF4") {
      currentState = MOVING_FORWARD;
      current_speed = SPEED_4;
      forward(left_motor, right_motor, current_speed);
      Serial.println("ACK:MF4");
    }
    else if (msg == "MF5") {
      currentState = MOVING_FORWARD;
      current_speed = SPEED_5;
      forward(left_motor, right_motor, current_speed);
      Serial.println("ACK:MF5");
    }
    // Move Right - Speed Levels 1-5
    else if (msg == "MR1") {
      currentState = TURNING_RIGHT;
      current_speed = SPEED_1;
      turnRight(current_speed);
      Serial.println("ACK:" + msg);
    }
    else if (msg == "MR2") {
      currentState = TURNING_RIGHT;
      current_speed = SPEED_2;
      turnRight(current_speed);
      Serial.println("ACK:MR2");
    }
    else if (msg == "MR3") {
      currentState = TURNING_RIGHT;
      current_speed = SPEED_3;
      turnRight(current_speed);
      Serial.println("ACK:MR3");
    }
    else if (msg == "MR4") {
      currentState = TURNING_RIGHT;
      current_speed = SPEED_4;
      turnRight(current_speed);
      Serial.println("ACK:MR4");
    }
    else if (msg == "MR5") {
      currentState = TURNING_RIGHT;
      current_speed = SPEED_5;
      turnRight(current_speed);
      Serial.println("ACK:MR5");
    }
    // Move Left - Speed Levels 1-5
    else if (msg == "ML1") {
      currentState = TURNING_LEFT;
      current_speed = SPEED_1;
      turnLeft(current_speed);
      Serial.println("ACK:" + msg);
    }
    else if (msg == "ML2") {
      currentState = TURNING_LEFT;
      current_speed = SPEED_2;
      turnLeft(current_speed);
      Serial.println("ACK:ML2");
    }
    else if (msg == "ML3") {
      currentState = TURNING_LEFT;
      current_speed = SPEED_3;
      turnLeft(current_speed);
      Serial.println("ACK:ML3");
    }
    else if (msg == "ML4") {
      currentState = TURNING_LEFT;
      current_speed = SPEED_4;
      turnLeft(current_speed);
      Serial.println("ACK:ML4");
    }
    else if (msg == "ML5") {
      currentState = TURNING_LEFT;
      current_speed = SPEED_5;
      turnLeft(current_speed);
      Serial.println("ACK:ML5");
    }
    // Move Backward - Speed Levels 1-5
    else if (msg == "MB1") {
      currentState = MOVING_BACKWARD;
      current_speed = SPEED_1;
      back(left_motor, right_motor, current_speed);
      Serial.println("ACK:MB1");
    }
    else if (msg == "MB2") {
      currentState = MOVING_BACKWARD;
      current_speed = SPEED_2;
      back(left_motor, right_motor, current_speed);
      Serial.println("ACK:MB2");
    }
    else if (msg == "MB3") {
      currentState = MOVING_BACKWARD;
      current_speed = SPEED_3;
      back(left_motor, right_motor, current_speed);
      Serial.println("ACK:MB3");
    }
    else if (msg == "MB4") {
      currentState = MOVING_BACKWARD;
      current_speed = SPEED_4;
      back(left_motor, right_motor, current_speed);
      Serial.println("ACK:MB4");
    }
    else if (msg == "MB5") {
      currentState = MOVING_BACKWARD;
      current_speed = SPEED_5;
      back(left_motor, right_motor, current_speed);
      Serial.println("ACK:MB5");
    }
    //Stop moving
    else if (msg == "MF0") {
      currentState = IDLE;
      brake(left_motor, right_motor);
      Serial.println("ACK:MF0");
    }
    // Activate Brush Motor
    else if (msg == "ABM") {
      BrushMotorOn();
      Serial.println("ACK:ABM");
    }
    // Deactivate Brush Motor
    else if (msg == "DBM") {
      BrushMotorOff();
      Serial.println("ACK:DBM");
    }
    // Drive Blind using only IR Sensors
    else if (msg == "DBI") {
      currentState = DRIVING_BLIND;
      stateStartTime = millis();
      Serial.println("ACK:DBI");
    }
    // Lower Tray
    else if (msg == "LTY") {
      LowerTray();
      Serial.println("ACK:LTY");
    }
    // Raise Tray
    else if (msg == "RTY") {
      RaiseTray();
      Serial.println("ACK:RTY");
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
      Serial.println("Arduino Received Unknown Command");
    }
  }
  
  // Execute current state behavior
  switch (currentState) {
    case DRIVING_BLIND:
      DriveBlind();
      break;
      
    case SEARCHING_YELLOW:
      YellowSearch();
      break;
      
    case IDLE:
      // Do nothing, motors already stopped
      break;
      
    case MOVING_FORWARD:
    case TURNING_LEFT:
    case TURNING_RIGHT:
    case MOVING_BACKWARD:
      // These are handled by the command that set the state
      // Motors continue in that direction until new command received
      break;
  }
}