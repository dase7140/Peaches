#include <SparkFun_TB6612.h>
#include <Wire.h>
#include <Adafruit_VL53L0X.h>

// =============================================================
// IR SENSOR SETUP  (unchanged from your version)
// =============================================================
#define TCAADDR 0x70
Adafruit_VL53L0X lox[5];

const char* irNames[5] = {
  "IRF_L", "IRF_R", "IRS", "IRL", "IRR"
};

void tcaSelect(uint8_t channel) {
  if (channel > 7) return;
  Wire.beginTransmission(TCAADDR);
  Wire.write(1 << channel);
  Wire.endTransmission();
  delay(3);
}

// =============================================================
// ULTRASONIC SENSOR SETUP  (unchanged from your version)
// =============================================================
struct UltrasonicSensor {
  int echoPin;
  int trigPin;
  const char* name;
};

UltrasonicSensor sensors[] = {
  {32, 33, "USF_L"},
  {34, 35, "USF_R"},
  {28, 29, "USS"},
  {24, 25, "USL"},
  {30, 31, "USR"}
};

const int numSensors = sizeof(sensors) / sizeof(sensors[0]);

long readUltrasonic(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  long duration = pulseIn(echoPin, HIGH, 30000);
  if (duration == 0) return -1;

  long distance = duration * 0.034 / 2;
  if (distance <= 0 || distance > 500) return -1;

  return distance;
}

// =============================================================
// MOTOR DRIVER SETUP
// =============================================================
#define BPWMA 41
#define BAIN1 43
#define BAIN2 45
#define BSTBY 47
#define BBIN1 49
#define BBIN2 51
#define BPWMB 53

const int offsetDrive = 1;
Motor motorLeft  = Motor(BAIN1, BAIN2, BPWMA, offsetDrive, BSTBY);
Motor motorRight = Motor(BBIN1, BBIN2, BPWMB, offsetDrive, BSTBY);

// Brush motor (shares channel with left motor driver board)
void spinnerMotor(int setting) {
  int pwm = 0;

  if (setting == -1) {
    // Reverse slowly
    digitalWrite(BAIN1, LOW);
    digitalWrite(BAIN2, HIGH);
    pwm = 80;
  } else {
    pwm = map(setting, 0, 9, 0, 255);
    digitalWrite(BAIN1, HIGH);
    digitalWrite(BAIN2, LOW);
  }

  analogWrite(BPWMA, pwm);
  digitalWrite(BSTBY, HIGH);
}

// =============================================================
// 7-CHAR COMMAND PARSER  (L0099D1 format)
// =============================================================
void handleCommand(String cmd) {
  

  char direction = cmd[0];        // L R B
  String angleStr = cmd.substring(1, 3); // 00–99 or XX
  String speedStr = cmd.substring(3, 5); // 00–99
  char tray = cmd[5];             // D or U
  char brush = cmd[6];            // 0–9 or R

  // Brush control
  if (brush == 'R')

  spinnerMotor(-1);
  else spinnerMotor(brush - '0');

  // (Tray servo control can be added here)

  // Convert speed percent to PWM
  int speedPct = speedStr.toInt();
  int pwm = map(speedPct, 0, 99, 0, 255);

  // Pivot turns
  if (angleStr == "XX") {
    if (direction == 'L') {
      motorLeft.drive(-pwm);
      motorRight.drive(pwm);
    } else if (direction == 'R') {
      motorLeft.drive(pwm);
      motorRight.drive(-pwm);
    }
    return;
  }

  // Arc turns
  int anglePct = angleStr.toInt();
  int delta = map(anglePct, 0, 99, 0, pwm);

  int L = pwm;
  int R = pwm;

  if (direction == 'L') {
    L -= delta;
    R += delta;
  }
  if (direction == 'R') {
    L += delta;
    R -= delta;
  }
  if (direction == 'B') {
    L = -L;
    R = -R;
  }
  motorLeft.drive(L);
  motorRight.drive(R);
}

// =============================================================
// SETUP
// =============================================================
void setup() {
  Serial.begin(115200);
  delay(1500);

  Wire.begin();

  // IR init
  for (uint8_t i = 0; i < 5; i++) {
    tcaSelect(i);
    if (!lox[i].begin()) {
      Serial.print("ERR: VL53L0X #");
      Serial.println(i);
    }
  }

  // Ultrasonic init
  for (int i = 0; i < numSensors; i++) {
    pinMode(sensors[i].trigPin, OUTPUT);
    pinMode(sensors[i].echoPin, INPUT);
  }

  pinMode(BPWMA, OUTPUT);
  pinMode(BAIN1, OUTPUT);
  pinMode(BAIN2, OUTPUT);
  pinMode(BSTBY, OUTPUT);
  pinMode(BPWMB, OUTPUT);
  pinMode(BBIN1, OUTPUT);
  pinMode(BBIN2, OUTPUT);
}

// =============================================================
// LOOP: continuous sensor data + command listening
// =============================================================
const int CMD_LEN = 7;

void loop() {
    static char cmdBuf[CMD_LEN + 1];
    static int cmdIndex = 0;

    while (Serial.available()) {
        char c = Serial.read();

        if (c == '\n' || c == '\r')
            continue;

        // Write byte safely
        cmdBuf[cmdIndex++] = c;

        // If buffer is full, process
        if (cmdIndex == CMD_LEN) {
            cmdBuf[CMD_LEN] = '\0';
            handleCommand(cmdBuf);

            Serial.print("CMD<:");
            Serial.print(cmdBuf);
            Serial.println(">");

            cmdIndex = 0;
        }

        // Prevent overflow on junk data
        if (cmdIndex > CMD_LEN) {
            cmdIndex = 0;
        }
    }

  // ----------------------------
  // Build SENS packet (your format)
  // ----------------------------
  String packet = "SENS ";

  // Ultrasonic
  for (int i = 0; i < numSensors; i++) {
    long d = readUltrasonic(sensors[i].trigPin, sensors[i].echoPin);
    packet += sensors[i].name;
    packet += "=";
    packet += d;
    packet += " ";
  }

  // IR sensors
  for (uint8_t i = 0; i < 5; i++) {
    tcaSelect(i);

    VL53L0X_RangingMeasurementData_t m;
    lox[i].rangingTest(&m, false);

    long dist;
    if (m.RangeStatus == 4) dist = -1;
    else {
      dist = m.RangeMilliMeter;
      if (dist < 20 || dist > 2000) dist = -1;
    }

    packet += irNames[i];
    packet += "=";
    packet += dist;
    packet += " ";
  }

  // Send sensor packet
  Serial.println(packet);
  

  delay(80);  // ~12.5 Hz
}
