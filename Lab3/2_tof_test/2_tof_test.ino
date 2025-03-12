#include <Wire.h>
#include "SparkFun_VL53L1X.h"

#define SHUTDOWN_PIN 8
#define INTERRUPT_PIN 3

SFEVL53L1X distanceSensor1;
SFEVL53L1X distanceSensor2(Wire, SHUTDOWN_PIN, INTERRUPT_PIN);

unsigned long loopCounter = 0;
unsigned long lastLoopTime = 0;
unsigned long lastSensor1Update = 0;
unsigned long sensor1Interval = 0;
void setup(void) {
  Wire.begin();
  Serial.begin(115200);
  // Setup for sensor2's shutdown pin
  pinMode(SHUTDOWN_PIN, OUTPUT);
  distanceSensor1.setI2CAddress(0xf5);
  // Bring sensor2 out of shutdown
  digitalWrite(SHUTDOWN_PIN, HIGH);
  delay(50);
  // Initialize both sensors
  if (distanceSensor1.begin() != 0) {
    Serial.println("Sensor1 failed to begin. Please check wiring. Freezing...");
    while (1) { delay(10); }
  }
  Serial.println("Sensor1 online!");
  if (distanceSensor2.begin() != 0) {
    Serial.println("Sensor2 failed to begin. Please check wiring. Freezing...");
    while (1) { delay(10); }
  }
  Serial.println("Sensor2 online!");
  distanceSensor1.setDistanceModeShort();
  distanceSensor2.setDistanceModeShort();
  // Start continuous ranging for both sensors
  distanceSensor1.startRanging();
  distanceSensor2.startRanging();
  
  lastLoopTime = millis();
}

void loop(void) {
  // Count loop iterations
  loopCounter++;

  // Print loop counter every 1 second
  unsigned long currentTime = millis();
  if (currentTime - lastLoopTime >= 1000) {
    Serial.print("Loop iterations in last second: ");
    Serial.println(loopCounter);
    loopCounter = 0;
    lastLoopTime = currentTime;
  }

  // Check sensor1 for new data
  if (distanceSensor1.checkForDataReady()) {
    int distance1 = distanceSensor1.getDistance();
    distanceSensor1.clearInterrupt();
    Serial.print("Sensor1 Distance: ");
    Serial.print(distance1);
  }
    // In continuous mode, do not stop ranging

    // // Calculate sensor update interval
    // if (lastSensor1Update != 0) {
    //   sensor1Interval = currentTime - lastSensor1Update;
    // }
    // lastSensor1Update = currentTime;
  if (distanceSensor2.checkForDataReady()) {
    int distance2 = distanceSensor2.getDistance();
    distanceSensor2.clearInterrupt();
    Serial.print("mm,  Sensor2 Distance: ");
    Serial.print(distance2);
  }
    // Serial.print(" mm, Update Interval: ");
    // Serial.print(sensor1Interval);
    // Serial.println(" ms");
    delay(500);
}
