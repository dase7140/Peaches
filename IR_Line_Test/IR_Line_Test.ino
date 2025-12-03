// ================
// IR Line Sensor Setup
// ================
int yellowPin = 33;  // Yellow IR line sensor
int whitePin = 32;   // White IR line sensor

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
    delay(10);
}

void LineSensorSetup(){
  pinMode(yellowPin, INPUT);
  pinMode(whitePin, INPUT);
  Serial.println("Line Sensors Initialized");
} 

void ReadLineSensors(int &yellowState, int &whiteState){
  yellowState = digitalRead(yellowPin);
  whiteState = digitalRead(whitePin);
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

void ReadOneIRDistance(int sensorIndex, int &distance) {
  tcaSelect(sensorIndex);
  VL53L0X_RangingMeasurementData_t measure;
  lox[sensorIndex].rangingTest(&measure, false);
  distance = measure.RangeMilliMeter;
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
const int SPEED_1 = 100;
const int SPEED_2 = 125;
const int SPEED_3 = 150;
const int SPEED_4 = 175;
const int SPEED_5 = 200;

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

void veerLeft(int leftSpd, int rightSpd){
  left_motor.drive(leftSpd);
  right_motor.drive(rightSpd);
}

void veerRight(int leftSpd, int rightSpd){
    left_motor.drive(leftSpd);
    right_motor.drive(rightSpd);
}


void drive_IR(int speed) {
  int yellowState = digitalRead(yellowPin);
  int whiteState = digitalRead(whitePin);
  int restTime = 25;
  int currentSpeed = speed;

  int frontLeft, frontRight;
  ReadFrontIRDistances(frontLeft, frontRight);
  
  // Check for critical obstacle
  if (frontLeft < 300|| frontRight < 300){
    currentSpeed = 120;
  }
  else {
    currentSpeed = speed;
  }

  if (frontLeft < 100 && frontRight > 100){
    turnLeft(speed);
  }
  else if (frontRight < 100 && frontLeft > 100){
    turnRight(speed);
  }
  else if (frontLeft < 100 && frontRight < 100){
    back(left_motor, right_motor, currentSpeed);
  }
  
  // Both sensors read 1 (both on line) - go forward
  if (yellowState == 1 && whiteState == 1) {
    forward(left_motor, right_motor, currentSpeed);
  }
  // White sensor triggered (0) - turn left
  else if (whiteState == 0 && yellowState == 1  ) {
    turnLeft(speed);
  }
  // Yellow sensor triggered (0) - turn right
  else if (yellowState == 0 && whiteState == 1) {
    turnRight(speed);
  }
  // Both sensors triggered - Faceplanting - Dont stop
  else if (yellowState == 0 && whiteState == 0) {
    // Do nothing
  }

  
}

// Arduino Safety Stop System
const int CRITICAL_STOP_DISTANCE = 115;  // mm - Emergency stop threshold
const unsigned long GRACE_PERIOD = 3000; // ms - 3 seconds to reposition after Pi override
unsigned long graceStartTime = 0;        // When Pi override began
bool safetyStopActive = false;           // True when Arduino has stopped due to obstacle
bool inGracePeriod = false;              // True during 3-second repositioning window

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

// Helper function to cap sensor readings (VL53L0X returns 8190+ when out of range)
int capDistance(int distance, int maxDistance) {
  if (distance >= 8000) {  // VL53L0X out-of-range indicator
    return maxDistance;
  }
  return min(distance, maxDistance);
}

void repositionArduino() {
  /*
   * Intelligent repositioning function when obstacle detected.
   * Arduino-based version that doesn't rely on Pi for repositioning.
   * 
   * Strategy:
   * 1. Read IR sensors to assess surroundings
   * 2. If back is clear (>100mm), reverse to create space
   * 3. Calculate left area vs right area and turn towards clearance
   */
  
  // Step 1: Read all IR sensor distances
  int IR_distances[numIRSensors];
  ReadAllIRDistances(IR_distances);
  
  // Extract individual sensor readings
  int leftDist = IR_distances[0];
  int frontRightDist = IR_distances[1];
  int frontLeftDist = IR_distances[2];
  int rightDist = IR_distances[3];
  int backDist = IR_distances[4];
  
  // Step 2: Check if back is clear and reverse if possible
  const int BACK_CLEAR_THRESHOLD = 100;  // mm
  int cappedBackDist = capDistance(backDist, 2000);
  
  if (cappedBackDist > BACK_CLEAR_THRESHOLD) {
    back(left_motor, right_motor, SPEED_3);
    delay(200);  // Reverse for 200ms
    brake(left_motor, right_motor);
    delay(200);
  }
  
  // Step 3: Cap sensor readings and calculate left and right clearance areas
  int frontLeftCapped = capDistance(frontLeftDist, 2000);
  int leftCapped = capDistance(leftDist, 2000);
  int frontRightCapped = capDistance(frontRightDist, 2000);
  int rightCapped = capDistance(rightDist, 2000);
  
  int leftArea = frontLeftCapped + leftCapped;
  int rightArea = frontRightCapped + rightCapped;
  
  // Step 4: Turn towards the direction with more clearance
  if (leftArea > rightArea) {
    turnLeft(SPEED_5);
    delay(200);  // Turn for 200ms
  } else {
    turnRight(SPEED_5);
    delay(200);  // Turn for 200ms
  }
  
  brake(left_motor, right_motor);
}



// Brush motor state (independent of movement)
bool brushMotorActive = false;

// Line following state
bool lineFollowingActive = false;

// Bridge mode state
bool bridgeModeActive = false;
unsigned long bridgeModeStartTime = 0;
const unsigned long BRIDGE_MODE_DURATION = 10000;  // 10 seconds in milliseconds
int desiredSpeed = SPEED_5;  // Default to fast speed

int loopCounter = 0;


// #############################################################
// ##### MAIN PROGRAM ##########################################
// #############################################################

void setup() {
  // Serial Communications
  CommsSetup();
  // IR Sensors
  IRSensorSetup();
  // Line Sensors
  LineSensorSetup();
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
    // Move Forward - Speed Levels 1-5 
    if (msg == "MF1") {
      safetyStopActive = false;
      inGracePeriod = false;  // Terminate grace period on new command
      lineFollowingActive = false;  // Disable line following
      current_speed = SPEED_1;
      forward(left_motor, right_motor, current_speed);
      Serial.println("ACK:MF1");
    }
    else if (msg == "MF2") {
      safetyStopActive = false;
      inGracePeriod = false;  // Terminate grace period on new command
      lineFollowingActive = false;  // Disable line following
      current_speed = SPEED_2;
      forward(left_motor, right_motor, current_speed);
      Serial.println("ACK:MF2");
    }
    else if (msg == "MF3") {
      safetyStopActive = false;
      inGracePeriod = false;  // Terminate grace period on new command
      lineFollowingActive = false;  // Disable line following
      current_speed = SPEED_3;
      forward(left_motor, right_motor, current_speed);
      Serial.println("ACK:MF3");
    }
    else if (msg == "MF4") {
      safetyStopActive = false;
      inGracePeriod = false;  // Terminate grace period on new command
      lineFollowingActive = false;  // Disable line following
      current_speed = SPEED_4;
      forward(left_motor, right_motor, current_speed);
      Serial.println("ACK:MF4");
    }
    else if (msg == "MF5") {
      safetyStopActive = false;
      inGracePeriod = false;  // Terminate grace period on new command
      lineFollowingActive = false;  // Disable line following
      current_speed = SPEED_5;
      forward(left_motor, right_motor, current_speed);
      Serial.println("ACK:MF5");
    }

    // Move Right - Speed Levels 1-5
    else if (msg == "MR1") {
      safetyStopActive = false;
      lineFollowingActive = false;
      current_speed = SPEED_1;
      turnRight(current_speed);
      Serial.println("ACK:MR1");
    }
    else if (msg == "MR2") {
      safetyStopActive = false;
      lineFollowingActive = false;
      current_speed = SPEED_2;
      turnRight(current_speed);
      Serial.println("ACK:MR2");
    }
    else if (msg == "MR3") {
      safetyStopActive = false;
      lineFollowingActive = false;
      current_speed = SPEED_3;
      turnRight(current_speed);
      Serial.println("ACK:MR3");
    }
    else if (msg == "MR4") {
      safetyStopActive = false;
      lineFollowingActive = false;
      current_speed = SPEED_4;
      turnRight(current_speed);
      Serial.println("ACK:MR4");
    }
    else if (msg == "MR5") {
      safetyStopActive = false;
      lineFollowingActive = false;
      current_speed = SPEED_5;
      turnRight(current_speed);
      Serial.println("ACK:MR5");
    }

    // Move Left - Speed Levels 1-5
    else if (msg == "ML1") {
      safetyStopActive = false;
      lineFollowingActive = false;
      current_speed = SPEED_1;
      turnLeft(current_speed);
      Serial.println("ACK:ML1");
    }
    else if (msg == "ML2") {
      safetyStopActive = false;
      lineFollowingActive = false;
      current_speed = SPEED_2;
      turnLeft(current_speed);
      Serial.println("ACK:ML2");
    }
    else if (msg == "ML3") {
      safetyStopActive = false;
      lineFollowingActive = false;
      current_speed = SPEED_3;
      turnLeft(current_speed);
      Serial.println("ACK:ML3");
    }
    else if (msg == "ML4") {
      safetyStopActive = false;
      lineFollowingActive = false;
      current_speed = SPEED_4;
      turnLeft(current_speed);
      Serial.println("ACK:ML4");
    }
    else if (msg == "ML5") {
      safetyStopActive = false;
      lineFollowingActive = false;
      current_speed = SPEED_5;
      turnLeft(current_speed);
      Serial.println("ACK:ML5");
    }

    // Veer left, speed levels 1-5 (right motor turns faster)
    else if (msg == "VL1") {
      safetyStopActive = false;
      inGracePeriod = false;
      lineFollowingActive = false;
      current_speed = SPEED_1;
      veerLeft(current_speed/2, current_speed);
      Serial.println("ACK:VL1");
    }
    else if (msg == "VL2") {
      safetyStopActive = false;
      inGracePeriod = false;
      lineFollowingActive = false;
      current_speed = SPEED_2;
      veerLeft(current_speed/2, current_speed);
      Serial.println("ACK:VL2");
    }    
    else if (msg == "VL3") {
      safetyStopActive = false;
      inGracePeriod = false;
      lineFollowingActive = false;
      current_speed = SPEED_3;
      veerLeft(current_speed/2, current_speed);
      Serial.println("ACK:VL3");
    }
    else if (msg == "VL4") {
      safetyStopActive = false;
      inGracePeriod = false;
      lineFollowingActive = false;
      current_speed = SPEED_4;
      veerLeft(current_speed/2, current_speed);
      Serial.println("ACK:VL4");
    }
    else if (msg == "VL5") {
      safetyStopActive = false;
      inGracePeriod = false;
      lineFollowingActive = false;
      current_speed = SPEED_5;
      veerLeft(current_speed/2, current_speed);
      Serial.println("ACK:VL5");
    }


    //veer right, speed levels 1-5, (left motor turns faster)
    else if (msg == "VR1") {
      safetyStopActive = false;
      inGracePeriod = false;
      lineFollowingActive = false;
      current_speed = SPEED_1;
      veerRight(current_speed, current_speed/2);
      Serial.println("ACK:VR1");
    }
    else if (msg == "VR2") {
      safetyStopActive = false;
      inGracePeriod = false;
      lineFollowingActive = false;
      current_speed = SPEED_2;
      veerRight(current_speed, current_speed/2);
      Serial.println("ACK:VR2");
    }    
    else if (msg == "VR3") {
      safetyStopActive = false;
      inGracePeriod = false;
      lineFollowingActive = false;
      current_speed = SPEED_3;
      veerRight(current_speed, current_speed/2);
      Serial.println("ACK:VR3");
    }
    else if (msg == "VR4") {
      safetyStopActive = false;
      inGracePeriod = false;
      lineFollowingActive = false;
      current_speed = SPEED_4;
      veerRight(current_speed, current_speed/2);
      Serial.println("ACK:VR4");
    }
    else if (msg == "VR5") {
      safetyStopActive = false;
      inGracePeriod = false;
      lineFollowingActive = false;
      current_speed = SPEED_5;
      veerRight(current_speed, current_speed/2);
      Serial.println("ACK:VR5");
    }


    // Move Backward - Speed Levels 1-5
    else if (msg == "MB1") {
      safetyStopActive = false;
      lineFollowingActive = false;
      current_speed = SPEED_1;
      back(left_motor, right_motor, current_speed);
      Serial.println("ACK:MB1");
    }
    else if (msg == "MB2") {
      safetyStopActive = false;
      lineFollowingActive = false;
      current_speed = SPEED_2;
      back(left_motor, right_motor, current_speed);
      Serial.println("ACK:MB2");
    }
    else if (msg == "MB3") {
      safetyStopActive = false;
      lineFollowingActive = false;
      current_speed = SPEED_3;
      back(left_motor, right_motor, current_speed);
      Serial.println("ACK:MB3");
    }
    else if (msg == "MB4") {
      safetyStopActive = false;
      lineFollowingActive = false;
      current_speed = SPEED_4;
      back(left_motor, right_motor, current_speed);
      Serial.println("ACK:MB4");
    }
    else if (msg == "MB5") {
      safetyStopActive = false;
      lineFollowingActive = false;
      current_speed = SPEED_5;
      back(left_motor, right_motor, current_speed);
      Serial.println("ACK:MB5");
    }

    //Stop moving
    else if (msg == "MF0") {
      safetyStopActive = false;
      lineFollowingActive = false;
      brake(left_motor, right_motor);
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

    // Read Line Sensor States
    else if (msg == "RLS") {
      Serial.println("ACK:RLS");
      int yellowState, whiteState;
      ReadLineSensors(yellowState, whiteState);
      Serial.print("LS:");
      Serial.print(yellowState);
      Serial.print(",");
      Serial.println(whiteState);
    }

    // Drive_IR - Enable line following mode
    else if (msg == "SD1") {
      safetyStopActive = false;
      bridgeModeActive = false;  // Reset bridge mode
      current_speed = SPEED_1;
      lineFollowingActive = true;
      Serial.println("ACK:SD1");
    }
    else if (msg == "SD2") {
      safetyStopActive = false;
      bridgeModeActive = false;  // Reset bridge mode
      current_speed = SPEED_2;
      lineFollowingActive = true;
      Serial.println("ACK:SD2");
    }
    else if (msg == "SD3") {
      safetyStopActive = false;
      bridgeModeActive = false;  // Reset bridge mode
      current_speed = SPEED_3;
      lineFollowingActive = true;
      Serial.println("ACK:SD3");
    }
    else if (msg == "SD4") {
      safetyStopActive = false;
      bridgeModeActive = false;  // Reset bridge mode
      current_speed = SPEED_4;
      lineFollowingActive = true;
      Serial.println("ACK:SD4");
    }
    else if (msg == "SD5") {
      safetyStopActive = false;
      bridgeModeActive = false;  // Reset bridge mode
      current_speed = SPEED_5;
      lineFollowingActive = true;
      Serial.println("ACK:SD5");
    }


    // Unknown Command
    else {
      Serial.println("Arduino Received Unknown Command");
    }
  }
  

  
  // If line following is active, continuously execute drive_IR with speed adjustment
  if (lineFollowingActive) {
    // Check if bridge mode timer is active
    if (bridgeModeActive) {
      unsigned long elapsedTime = millis() - bridgeModeStartTime;
      
      if (elapsedTime >= BRIDGE_MODE_DURATION) {
        // Timer expired - exit bridge mode and return to normal speed
        bridgeModeActive = false;
        bridgeModeStartTime = 0;
        current_speed = SPEED_5;
      } else {
        // Still in bridge mode - stay at slow speed
        current_speed = SPEED_1;
      }
    } else {

    if (loopCounter++ >= 5) {
        loopCounter = 0;
        // Not in bridge mode - check IR sensors sequentially for bridge detection
        // Stop checking early if any sensor reads < 500mm (optimization)
        int IR_distances[numIRSensors];
        ReadOneIRDistance(0, IR_distances[0]); // Left
        if (IR_distances[0] >= 500) { 
            ReadOneIRDistance(1, IR_distances[1]); // Front Right
            if (IR_distances[1] >= 500) {
            ReadOneIRDistance(2, IR_distances[2]); // Front Left
            if (IR_distances[2] >= 500) {
                ReadOneIRDistance(3, IR_distances[3]); // Right
                if (IR_distances[3] >= 500) {
                ReadOneIRDistance(4, IR_distances[4]); // Back
                if (IR_distances[4] >= 500) {
                    // All sensors read >500mm - bridge detected!
                    current_speed = SPEED_1;
                    bridgeModeActive = true;
                    bridgeModeStartTime = millis();
                }
                }
            }
            }
        }
    }
    loopCounter++;
      // If not entering bridge mode, use normal speed
      if (!bridgeModeActive) {
        current_speed = SPEED_5;
      }
    }
    
    // Execute line following with current speed
    drive_IR(current_speed);
  }
}