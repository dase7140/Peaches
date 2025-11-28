#include <Wire.h>
#include <Adafruit_VL53L0X.h>

Adafruit_VL53L0X lox[5];  // for up to 5 sensors
#define TCAADDR 0x70

// Select the TCA channel (0–7)
void tcaSelect(uint8_t channel) {
  if (channel > 7) return;
  Wire.beginTransmission(TCAADDR);
  Wire.write(1 << channel);
  Wire.endTransmission();
  delay(10);
}

void setup() {
  Serial.begin(115200);
  Wire.begin();

  Serial.println("Initializing multiple VL53L0X sensors...");

  for (uint8_t i = 0; i < 5; i++) {
    tcaSelect(i);
    if (!lox[i].begin()) {
      Serial.print("Failed to boot VL53L0X on channel ");
      Serial.println(i);
      while(1);
    }
    Serial.print("VL53L0X sensor initialized on channel ");
    Serial.println(i);
  }
}

void loop() {
  for (uint8_t i = 0; i < 5; i++) {
    tcaSelect(i);

    VL53L0X_RangingMeasurementData_t measure;
    lox[i].rangingTest(&measure, false);

    Serial.print("Sensor "); Serial.print(i); Serial.print(": ");
    if (measure.RangeStatus != 4) {
      Serial.print(measure.RangeMilliMeter);
      Serial.println(" mm");
    } else {
      Serial.println("Out of range");
    }
  }

  Serial.println();
  //delay(500);
}
