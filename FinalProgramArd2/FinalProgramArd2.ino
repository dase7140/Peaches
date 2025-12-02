// ================
// IR Sensor Setup - VL53L0X Time-of-Flight Distance Sensors with TCA9548A I2C Multiplexer
// ================

#include <Wire.h>
#include <Adafruit_VL53L0X.h>

Adafruit_VL53L0X lox[5];  // for up to 5 sensors

// Left - 0
// Front Right - 1
// Front Left - 2
// Right - 3  
// Back - 4

#define TCAADDR 0x70
#define numIRSensors 5

void tcaSelect(uint8_t channel) {
  if (channel > 7) return;
    Wire.beginTransmission(TCAADDR);
    Wire.write(1 << channel);
    Wire.endTransmission();
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

void ReadFrontIRDistances(int &leftFront, int &rightFront) {
  tcaSelect(2); // Front Left
  VL53L0X_RangingMeasurementData_t measureLeft;
  lox[2].rangingTest(&measureLeft, false);
  leftFront = measureLeft.RangeMilliMeter;

  tcaSelect(1); // Front Right
  VL53L0X_RangingMeasurementData_t measureRight;
  lox[1].rangingTest(&measureRight, false);
  rightFront = measureRight.RangeMilliMeter;
}

// =============================================================
// Drive Motor Setup - SparkFun TB6612FNG
// =============================================================



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

const int SPEED_ARRAY[6] = {0,SPEED_1,SPEED_2,SPEED_3,SPEED_4,SPEED_5}; 

int current_speed = SPEED_2; // Default speed

Motor left_motor = Motor(AIN1, AIN2, PWMA, offsetA, STBY);
Motor right_motor = Motor(BIN1, BIN2, PWMB, offsetB, STBY); 

void DriveMotorSetup(){
    brake(left_motor, right_motor); // Ensure motors are stopped at startup
    Serial.println("Drive Motors Initialized");
}


// ============================================================
// Brush Motor Setup - Pololu DRV8263H
// ============================================================

// Brush Motor Setup 
// Brush Motor Driver Pins
#define brush_IN1 7
#define brush_IN2 6

void BrushMotorSetup(){
  pinMode(brush_IN1, OUTPUT);
  pinMode(brush_IN2, OUTPUT);
  digitalWrite(brush_IN1, HIGH);
  digitalWrite(brush_IN2, HIGH);
  Serial.println("Brush Motor Initialized");
}

// ============================================================
// Tray Servo Setup - 20G Hobby Servo
// ============================================================

// Tray Servo Setup
#include <Servo.h>   // Basic Arduino Servo Library

// Servo Pins
#define tray_servo_pin 9

Servo tray_servo; 

int down = 90;   // Lowered position
int up = 165;       // Raised position

void TrayServoSetup(){
  tray_servo.attach(tray_servo_pin);
  tray_servo.write(down); // Set to initial position
  Serial.println("Tray Servo Initialized");
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
// =============================================================

void CommsSetup(){
  Serial.begin(115200);
  delay(1500); // Wait for serial to initialize
  Serial.println("Communications Initialized");
} 




// ##################################
// ##### Combination Functions ######
// ##################################

int left_speed_target = 0;
int right_speed_target = 0;
int left_speed_current = 0;
int right_speed_current = 0;


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
  left_speed_target = -spd;
  right_speed_target = spd;
  // left_motor.drive(-spd);
  // right_motor.drive(spd);
}

void turnRight(int spd){
  left_speed_target = spd;
  right_speed_target = -spd;
  // left_motor.drive(spd);
  // right_motor.drive(-spd);
}

void veerLeft(int leftSpd, int rightSpd){
  left_speed_target = leftSpd;
  right_speed_target = rightSpd;
  // left_motor.drive(leftSpd);
  // right_motor.drive(rightSpd);
}

void veerRight(int leftSpd, int rightSpd){
  left_speed_target = leftSpd;
  right_speed_target = rightSpd;
    // left_motor.drive(leftSpd);
    // right_motor.drive(rightSpd);
}

// Arduino Safety Stop System
const int CRITICAL_STOP_DISTANCE = 200;  // mm - Emergency stop threshold
const unsigned long GRACE_PERIOD = 5000; // ms - 5 seconds to reposition after Pi override
unsigned long graceStartTime = 0;        // When Pi override began
bool safetyStopActive = false;           // True when Arduino has stopped due to obstacle
bool inGracePeriod = false;              // True during 5-second repositioning window

void checkFrontObstacles() {
  // Check if we're in grace period (Pi is repositioning)
  if (inGracePeriod) {
    unsigned long currentTime = millis();
    if (currentTime - graceStartTime >= GRACE_PERIOD) {
      // Grace period expired, re-enable safety checks
      inGracePeriod = false;
    } else {
      // Still in grace period - skip obstacle checks
      return;
    }
  }
  
  // Read front sensors
  int frontLeft, frontRight;
  ReadFrontIRDistances(frontLeft, frontRight);
  
  // Check for critical obstacle
  if (frontLeft < CRITICAL_STOP_DISTANCE || frontRight < CRITICAL_STOP_DISTANCE) {
    if (!safetyStopActive) {
      // Emergency stop!
      brake(left_motor, right_motor);
      safetyStopActive = true;
      // Start grace period after safety stop
      inGracePeriod = true;
      graceStartTime = millis();
      Serial.println("ESTOP");
    }
  } else {
    // Path is clear - reset safety flag
    safetyStopActive = false;
  }
}



// Brush motor state (independent of movement)
bool brushMotorActive = false;




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

  // Moving Tray Servo to confirm Robot is ready
  LowerTray();
  delay(500);
  RaiseTray();
  delay(500);
  Serial.println("Arduino Setup Complete: Ready to receive commands...");
}


int ACC_INCREMENT = 40; 
int CUTOFF_BAND = 100; 

int sign(int num){  return (num < 0) ? -1 : 1;}



void loop() {
  //updates the left motor target speed to approach the set speed
  if(left_speed_current != left_speed_target){
    if(abs(left_speed_target-left_speed_current) > ACC_INCREMENT){
      if((left_speed_current - left_speed_target)>0){
        left_speed_current -= ACC_INCREMENT;
      }
      else {
        left_speed_current += ACC_INCREMENT;
      }
    }
    else{
      left_speed_current = left_speed_target;
    }
    if(left_speed_target != 0){
      // If we're requesting a direction change, or current is inside deadband,
      // jump to the cutoff in the target direction to reliably pass the deadzone.
      if (abs(left_speed_current) < CUTOFF_BAND) {
        left_speed_current = sign(left_speed_target) * CUTOFF_BAND;
      }
    }
    else {
      left_speed_current = 0;
    }
  }

  if(right_speed_current != right_speed_target){
    if(abs(right_speed_target-right_speed_current) > ACC_INCREMENT){
      if((right_speed_current - right_speed_target)>0){
        right_speed_current -= ACC_INCREMENT;
      }
      else {
        right_speed_current += ACC_INCREMENT;
      }
    }
    else{
      right_speed_current = right_speed_target;
    }
    if(right_speed_target != 0){
      // If we're requesting a direction change, or current is inside deadband,
      // jump to the cutoff in the target direction to reliably pass the deadzone.
      if (abs(right_speed_current) < CUTOFF_BAND) {
        right_speed_current = sign(right_speed_target) * CUTOFF_BAND;
      }
    }  
    else{
      right_speed_current = 0;
    }
  }

  left_motor.drive(left_speed_current);
  right_motor.drive(right_speed_current);  

  // Process serial commands FIRST before executing states
  if (Serial.available() > 0){
    String msg = Serial.readStringUntil('\n');
    msg.trim(); // Remove any leading/trailing whitespace

    // Ignore empty messages
    if (msg.length() == 0){
      return; 
    }
    
    // Process commands and change state
    // Move Forward - Speed Levels 1-5 
    if (msg == "MF1") {
      safetyStopActive = false;
      inGracePeriod = false;  // Terminate grace period on new command
      current_speed = SPEED_1;
      left_speed_target = current_speed;
      right_speed_target = current_speed;
      //forward(left_motor, right_motor, current_speed);
      Serial.println("ACK:MF1");
    }
    else if (msg == "MF2") {
      safetyStopActive = false;
      inGracePeriod = false;  // Terminate grace period on new command
      current_speed = SPEED_2;
      left_speed_target = current_speed;
      right_speed_target = current_speed;
      //forward(left_motor, right_motor, current_speed);
      Serial.println("ACK:MF2");
    }
    else if (msg == "MF3") {
      safetyStopActive = false;
      inGracePeriod = false;  // Terminate grace period on new command
      current_speed = SPEED_3;
      left_speed_target = current_speed;
      right_speed_target = current_speed;
      //forward(left_motor, right_motor, current_speed);
      Serial.println("ACK:MF3");
    }
    else if (msg == "MF4") {
      safetyStopActive = false;
      inGracePeriod = false;  // Terminate grace period on new command
      current_speed = SPEED_4;
      left_speed_target = current_speed;
      right_speed_target = current_speed;
      //forward(left_motor, right_motor, current_speed);
      Serial.println("ACK:MF4");
    }
    else if (msg == "MF5") {
      safetyStopActive = false;
      inGracePeriod = false;  // Terminate grace period on new command
      current_speed = SPEED_5;
      left_speed_target = current_speed;
      right_speed_target = current_speed;
      //forward(left_motor, right_motor, current_speed);
      Serial.println("ACK:MF5");
    }

    // Move Right - Speed Levels 1-5
    else if (msg == "MR1") {
      safetyStopActive = false;
      current_speed = SPEED_1;
      turnRight(current_speed);
      Serial.println("ACK:MR1");
    }
    else if (msg == "MR2") {
      safetyStopActive = false;
      current_speed = SPEED_2;
      turnRight(current_speed);
      Serial.println("ACK:MR2");
    }
    else if (msg == "MR3") {
      safetyStopActive = false;
      current_speed = SPEED_3;
      turnRight(current_speed);
      Serial.println("ACK:MR3");
    }
    else if (msg == "MR4") {
      safetyStopActive = false;
      current_speed = SPEED_4;
      turnRight(current_speed);
      Serial.println("ACK:MR4");
    }
    else if (msg == "MR5") {
      safetyStopActive = false;
      current_speed = SPEED_5;
      turnRight(current_speed);
      Serial.println("ACK:MR5");
    }

    // Move Left - Speed Levels 1-5
    else if (msg == "ML1") {
      safetyStopActive = false;
      current_speed = SPEED_1;
      turnLeft(current_speed);
      Serial.println("ACK:ML1");
    }
    else if (msg == "ML2") {
      safetyStopActive = false;
      current_speed = SPEED_2;
      turnLeft(current_speed);
      Serial.println("ACK:ML2");
    }
    else if (msg == "ML3") {
      safetyStopActive = false;
      current_speed = SPEED_3;
      turnLeft(current_speed);
      Serial.println("ACK:ML3");
    }
    else if (msg == "ML4") {
      safetyStopActive = false;
      current_speed = SPEED_4;
      turnLeft(current_speed);
      Serial.println("ACK:ML4");
    }
    else if (msg == "ML5") {
      safetyStopActive = false;
      current_speed = SPEED_5;
      turnLeft(current_speed);
      Serial.println("ACK:ML5");
    }

    // Veer left, speed levels 1-5 (right motor turns faster)
    else if (msg == "VL1") {
      safetyStopActive = false;
      inGracePeriod = false;
      veerLeft(max(SPEED_ARRAY[1]/2,CUTOFF_BAND+1),SPEED_ARRAY[1]);
      Serial.println("ACK:VL1");
    }
    else if (msg == "VL2") {
      safetyStopActive = false;
      inGracePeriod = false;
      veerLeft(max(SPEED_ARRAY[2]/2,CUTOFF_BAND+1),SPEED_ARRAY[2]);
      Serial.println("ACK:VL2");
    }    
    else if (msg == "VL3") {
      safetyStopActive = false;
      inGracePeriod = false;
      veerLeft(max(SPEED_ARRAY[3]/2,CUTOFF_BAND+1),SPEED_ARRAY[3]);
      Serial.println("ACK:VL3");
    }
    else if (msg == "VL4") {
      safetyStopActive = false;
      inGracePeriod = false;
      veerLeft(max(SPEED_ARRAY[4]/2,CUTOFF_BAND+1),SPEED_ARRAY[4]);
      Serial.println("ACK:VL4");
    }
    else if (msg == "VL5") {
      safetyStopActive = false;
      inGracePeriod = false;
      veerLeft(max(SPEED_ARRAY[5]/2,CUTOFF_BAND+1),SPEED_ARRAY[5]);
      Serial.println("ACK:VL5");
    }


    //veer right, speed levels 1-5, (left motor turns faster)
    else if (msg == "VR1") {
      safetyStopActive = false;
      inGracePeriod = false;
      veerRight(SPEED_ARRAY[1],max(SPEED_ARRAY[1]/2,CUTOFF_BAND+1));
      Serial.println("ACK:VR1");
    }
    else if (msg == "VR2") {
      safetyStopActive = false;
      inGracePeriod = false;
      veerRight(SPEED_ARRAY[2],max(SPEED_ARRAY[2]/2,CUTOFF_BAND+1));
      Serial.println("ACK:VR2");
    }    
    else if (msg == "VR3") {
      safetyStopActive = false;
      inGracePeriod = false;
      veerRight(SPEED_ARRAY[3],max(SPEED_ARRAY[3]/2,CUTOFF_BAND+1));
      Serial.println("ACK:VR3");
    }
    else if (msg == "VR4") {
      safetyStopActive = false;
      inGracePeriod = false;
      veerRight(SPEED_ARRAY[4],max(SPEED_ARRAY[4]/2,CUTOFF_BAND+1));
      Serial.println("ACK:VR4");
    }
    else if (msg == "VR5") {
      safetyStopActive = false;
      inGracePeriod = false;
      veerRight(SPEED_ARRAY[5],max(SPEED_ARRAY[5]/2,CUTOFF_BAND+1));
      Serial.println("ACK:VR5");
    }


    // Move Backward - Speed Levels 1-5
    else if (msg == "MB1") {
      safetyStopActive = false;
      current_speed = SPEED_1;
      left_speed_target = -current_speed;
      right_speed_target = -current_speed;
      //back(left_motor, right_motor, current_speed);
      Serial.println("ACK:MB1");
    }
    else if (msg == "MB2") {
      safetyStopActive = false;
      current_speed = SPEED_2;
      left_speed_target = -current_speed;
      right_speed_target = -current_speed;
      //back(left_motor, right_motor, current_speed);
      Serial.println("ACK:MB2");
    }
    else if (msg == "MB3") {
      safetyStopActive = false;
      current_speed = SPEED_3;
      left_speed_target = -current_speed;
      right_speed_target = -current_speed;
      //back(left_motor, right_motor, current_speed);
      Serial.println("ACK:MB3");
    }
    else if (msg == "MB4") {
      safetyStopActive = false;
      current_speed = SPEED_4;
      left_speed_target = -current_speed;
      right_speed_target = -current_speed;
      //back(left_motor, right_motor, current_speed);
      Serial.println("ACK:MB4");
    }
    else if (msg == "MB5") {
      safetyStopActive = false;
      current_speed = SPEED_5;
      left_speed_target = -current_speed;
      right_speed_target = -current_speed;
      //back(left_motor, right_motor, current_speed);
      Serial.println("ACK:MB5");
    }

    //Stop moving
    else if (msg == "MF0") {
      safetyStopActive = false;
      left_speed_target = 0;
      right_speed_target = 0;
      //brake(left_motor, right_motor);
      Serial.println("ACK:MF0");
    }

    // Activate Brush Motor
    else if (msg == "ABM") {
      Serial.println("ACK:ABM");
      BrushMotorOn();
    }
    // Deactivate Brush Motor
    else if (msg == "DBM") {
      Serial.println("ACK:DBM");
      BrushMotorOff();
    }

    // Lower Tray
    else if (msg == "LTY") {
      Serial.println("ACK:LTY");
      LowerTray();
    }
    // Raise Tray
    else if (msg == "RTY") {
      Serial.println("ACK:RTY");
      RaiseTray();
    }

    // Read IR Sensor Distances
    else if (msg == "RIS") {
      Serial.println("ACK:RIS");
      int IR_distances[numIRSensors];
      ReadAllIRDistances(IR_distances);
      Serial.print("IR:");
      for (int i = 0; i < numIRSensors; i++) {
        Serial.print(IR_distances[i]);
        if (i < numIRSensors - 1) {
          Serial.print(",");
        }
      }
      Serial.println();
    }

    // Unknown Command
    else {
      Serial.println("Arduino Received Unknown Command");
    }
  }
  
  // Continuously check for front obstacles (Arduino safety layer)
  checkFrontObstacles();
  
}