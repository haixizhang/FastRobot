#include "BLECStringCharacteristic.h"
#include "EString.h"
#include "RobotCommand.h"
#include <ArduinoBLE.h>

//////////// BLE UUIDs ////////////
#define BLE_UUID_TEST_SERVICE "ae3527ce-10d8-45c0-9f82-2158e073efba"
#define BLE_UUID_RX_STRING "9750f60b-9c9c-4158-b620-02ec9521cd99"
#define BLE_UUID_TX_FLOAT "27616294-3063-4ecc-b60b-3470ddef2938"
#define BLE_UUID_TX_STRING "f235a225-6735-4d73-94cb-ee5dfce9ba83"
//////////// BLE UUIDs ////////////

//////////// Global Variables ////////////
BLEService testService(BLE_UUID_TEST_SERVICE);
BLECStringCharacteristic rx_characteristic_string(BLE_UUID_RX_STRING, BLEWrite, MAX_MSG_SIZE);
BLEFloatCharacteristic tx_characteristic_float(BLE_UUID_TX_FLOAT, BLERead | BLENotify);
BLECStringCharacteristic tx_characteristic_string(BLE_UUID_TX_STRING, BLERead | BLENotify, MAX_MSG_SIZE);

static double lastSendTime = 0;
static const double NOTIF_INTERVAL = 10;  // milliseconds between sends
// RX
RobotCommand robot_cmd(":|");
// TX
EString tx_estring_value;
float tx_float_value = 0.0;

long interval = 500;
static long previousMillis = 0;
unsigned long currentMillis = 0;

// IMU Libraries & Globals
#include "ICM_20948.h" 
#define SERIAL_PORT Serial
#define WIRE_PORT Wire
#define AD0_VAL 1
ICM_20948_I2C myICM;

bool recordingActive = false;
unsigned long recordStartTime = 0;
unsigned long currentTime = 0;
const int MAX_SAMPLES = 2000;
unsigned long timeStamps[MAX_SAMPLES];
float pitchArray[MAX_SAMPLES], rollArray[MAX_SAMPLES];
int sampleIndex = 0;
int failed_attempt = 0;

// ToF Libraries & Globals
#include <Wire.h>
#include "SparkFun_VL53L1X.h"
#define SHUTDOWN_PIN 8
#define INTERRUPT_PIN 3
SFEVL53L1X distanceSensor1;
SFEVL53L1X distanceSensor2(Wire, SHUTDOWN_PIN, INTERRUPT_PIN);
static int tof1Distances[MAX_SAMPLES];
static int tof2Distances[MAX_SAMPLES];
static unsigned long tofTimes[MAX_SAMPLES];
int dataSampleIndex = 0;
bool recordingAll = false;
unsigned long recordAllStartTime = 0;

// Motor pins
const int MOTOR_BLUE_FWD = 6;   
const int MOTOR_BLUE_REV = 7;   
const int MOTOR_OTHER_FWD = 13; 
const int MOTOR_OTHER_REV = 14; 

// Motor calibration
const int MAX_PWM_OTHER = 65;                      
const int MAX_PWM_BLUE = (int)(MAX_PWM_OTHER * 34.0 / 40.0); 

/***** PID and Extrapolation Globals *****/

// PID gains
float Kp = 0.0f;
float Ki = 0.0f;
float Kd = 0.0f;

// Arrays to store PID logs
static const int MAX_PID_SAMPLES = 1000;
int   pidTimeArray[MAX_PID_SAMPLES];       
float pidErrorArray[MAX_PID_SAMPLES];      
float pidOutputArray[MAX_PID_SAMPLES];     
// We'll store the raw sensor reading here:
float pidRawDistArray[MAX_PID_SAMPLES];
// And the extrapolated distance here:
float pidExtrapolatedDistArray[MAX_PID_SAMPLES];
int   pidMotorPWMArray[MAX_PID_SAMPLES];

int pidDataIndex = 0;

bool pidActive = false;
unsigned long pidTestDuration = 5000; 
unsigned long pidStartTime = 0;
float pidSetpoint = 20.0;

// For safety if BLE disconnects
bool bleConnected = false;

// ---------- Global Variables for Extrapolation -----------
float distanceSensor1_cur = 0.0f, distanceSensor1_prev = 0.0f;
unsigned long timeSensor1_cur = 0, timeSensor1_prev = 0;
bool firstReading = true;  // to handle startup gracefully

// NEW global variables to store both raw and extrapolated values:
float lastRawDistance = 0.0f;
float lastExtrapolatedDistance = 0.0f;

// Define a maximum value for the integrator to prevent wind-up
#define MAX_INTEGRAL 800.0f
// Flag to enable or disable anti-windup
bool antiWindupEnabled = true;

//////////// Command Types ////////////
enum CommandTypes {
  GET_PITCH_ROLL,
  GET_FROM_GYROSCOPE,
  STORE_IMU_DATA,
  SEND_IMU_DATA,
  TOF_DATA,
  STORE_DUAL_TOF_IMU_DATA,
  SEND_DUAL_TOF_IMU_DATA,
  SET_PID_GAINS,
  START_PID_CONTROL,
  GET_PID_DATA,
};

/****************************************************************
 * Helper Functions
 ***************************************************************/
void stopMotors() {
  analogWrite(MOTOR_BLUE_FWD, 0);
  analogWrite(MOTOR_BLUE_REV, 0);
  analogWrite(MOTOR_OTHER_FWD, 0);
  analogWrite(MOTOR_OTHER_REV, 0);
}

int mapPIDToMotor(float pidVal) {
  if (pidVal > 255.0f) { pidVal = 255.0f; }
  else if (pidVal < -255.0f) { pidVal = -255.0f; }
  return (int)pidVal;
}

void setMotorSpeed(int motorCmd) {
  int absCmd = abs(motorCmd);
  if (absCmd > 100) absCmd = 100;
  int pwmRight = absCmd;
  int pwmLeft  = (int)(absCmd * 32.0 / 40.0);

  if (motorCmd > 0) {
    analogWrite(MOTOR_BLUE_FWD, pwmLeft);
    analogWrite(MOTOR_BLUE_REV, 0);
    analogWrite(MOTOR_OTHER_FWD, pwmRight);
    analogWrite(MOTOR_OTHER_REV, 0);
  } else if (motorCmd < 0) {
    analogWrite(MOTOR_BLUE_FWD, 0);
    analogWrite(MOTOR_BLUE_REV, pwmLeft);
    analogWrite(MOTOR_OTHER_FWD, 0);
    analogWrite(MOTOR_OTHER_REV, pwmRight);
  } else {
    stopMotors();
  }
}

/****************************************************************
 * Extrapolation-Enabled Distance Function
 * 
 * This function updates two global values:
 *  - lastRawDistance: the most recent raw sensor reading.
 *  - lastExtrapolatedDistance: if no new data is available,
 *    the extrapolated distance computed from the previous two readings.
 ***************************************************************/
float getDistanceSensor1Extrapolated() {
  bool newData = distanceSensor1.checkForDataReady();
  if (newData) {
    // Shift previous values
    distanceSensor1_prev = distanceSensor1_cur;
    timeSensor1_prev = timeSensor1_cur;
    int dist = distanceSensor1.getDistance();
    distanceSensor1.clearInterrupt();
    if (dist > 0) {
      distanceSensor1_cur = (float)dist;
      timeSensor1_cur = millis();
    }
    if (firstReading) {
      distanceSensor1_prev = distanceSensor1_cur;
      timeSensor1_prev = timeSensor1_cur;
      firstReading = false;
    }
    // For new data, raw and extrapolated values are the same.
    lastRawDistance = distanceSensor1_cur;
    lastExtrapolatedDistance = distanceSensor1_cur;
    return distanceSensor1_cur;
  } else {
    // No new data: raw value remains the last sensor reading...
    lastRawDistance = distanceSensor1_cur;
    // ...and extrapolate forward.
    unsigned long now_ms = millis();
    float dt = (now_ms - timeSensor1_cur) / 1000.0f;
    float dt_sens = (timeSensor1_cur - timeSensor1_prev) / 1000.0f;
    if (dt_sens > 0.0f) {
      float slope = (distanceSensor1_cur - distanceSensor1_prev) / dt_sens;
      lastExtrapolatedDistance = distanceSensor1_cur + slope * dt;
      return lastExtrapolatedDistance;
    } else {
      lastExtrapolatedDistance = distanceSensor1_cur;
      return distanceSensor1_cur;
    }
  }
}

/****************************************************************
 * The PID Loop
 * 
 * Uses the extrapolated distance for control while logging both
 * the raw sensor reading and the extrapolated distance.
 ***************************************************************/
// void runPIDIteration() {
//   static unsigned long lastPIDTime = millis();
//   unsigned long now = millis();
//   float dt = (now - lastPIDTime) / 1000.0f;
//   lastPIDTime = now;

//   // Get the extrapolated distance; globals are updated in the function.
//   float extrapolatedDistance = getDistanceSensor1Extrapolated();
//   // Read the raw sensor value (last valid reading)
//   float rawDistance = lastRawDistance;
  
//   // Use the extrapolated distance for PID computation.
//   float error = pidSetpoint - extrapolatedDistance;
//   static float integralTerm = 0.0f;
//   integralTerm += error * dt;
//   static float previousError = 0.0f;
//   float derivativeTerm = (error - previousError) / dt;
//   previousError = error;
//   float output = Kp * error + Ki * integralTerm + Kd * derivativeTerm;
//   int motorCmd = mapPIDToMotor(output);
//   setMotorSpeed(motorCmd);

//   // Log data into arrays.
//   if (pidDataIndex < MAX_PID_SAMPLES) {
//     pidTimeArray[pidDataIndex] = (int)now;
//     pidErrorArray[pidDataIndex] = error;
//     pidOutputArray[pidDataIndex] = output;
//     pidMotorPWMArray[pidDataIndex] = motorCmd;
//     // Store raw sensor reading and extrapolated distance separately.
//     pidRawDistArray[pidDataIndex] = rawDistance;
//     pidExtrapolatedDistArray[pidDataIndex] = extrapolatedDistance;
//     pidDataIndex++;
//   }
// }

void runPIDIteration() {
  static unsigned long lastPIDTime = millis();
  unsigned long now = millis();
  float dt = (now - lastPIDTime) / 1000.0f;  // dt in seconds
  lastPIDTime = now;

  // Get the current distance using your extrapolation function
  float currentDistance = getDistanceSensor1Extrapolated();

  // Compute error (setpoint minus current distance)
  float error = pidSetpoint - currentDistance;

  // PID terms:
  static float integralTerm = 0.0f;
  static float previousError = 0.0f;

  // --- Integrator Update with Anti-Windup Protection ---
  if (antiWindupEnabled) {
    integralTerm += error * dt;
    // Clamp the integrator term so that it doesn't wind up beyond a maximum limit
    if (integralTerm > MAX_INTEGRAL)
      integralTerm = MAX_INTEGRAL;
    else if (integralTerm < -MAX_INTEGRAL)
      integralTerm = -MAX_INTEGRAL;
  } else {
    // No wind-up protection (for demonstration/comparison)
    integralTerm += error * dt;
  }

  // Derivative calculation
  float derivativeTerm = (error - previousError) / dt;
  previousError = error;

  // Compute the PID controller output
  float output = Kp * error + Ki * integralTerm + Kd * derivativeTerm;
  int motorCmd = mapPIDToMotor(output);
  setMotorSpeed(motorCmd);

  // Log data (this code logs your PID parameters, raw distance, etc.)
  if (pidDataIndex < MAX_PID_SAMPLES) {
    pidTimeArray[pidDataIndex] = (int)now;
    pidErrorArray[pidDataIndex] = error;
    pidOutputArray[pidDataIndex] = output;
    pidMotorPWMArray[pidDataIndex] = motorCmd;
    pidRawDistArray[pidDataIndex] = lastRawDistance;         // Raw sensor reading
    pidExtrapolatedDistArray[pidDataIndex] = lastExtrapolatedDistance; // Extrapolated distance
    pidDataIndex++;
  }
}



/****************************************************************
 * Handle BLE Commands
 ***************************************************************/
void handle_command() {
  robot_cmd.set_cmd_string(rx_characteristic_string.value(), rx_characteristic_string.valueLength());
  bool success;
  int cmd_type = -1;
  success = robot_cmd.get_command_type(cmd_type);
  if (!success) { return; }

  switch (cmd_type) {
    case GET_PITCH_ROLL:
      {
        myICM.getAGMT();
        float ax = myICM.accX() / 1000.0f;
        float ay = myICM.accY() / 1000.0f;
        float az = myICM.accZ() / 1000.0f;
        float pitchDeg = atan2(ax, az) * 180.0f / M_PI;
        float rollDeg  = atan2(ay, az) * 180.0f / M_PI;
        static float filteredPitch = pitchDeg;
        static float filteredRoll  = rollDeg;
        static unsigned long last_time = micros();
        unsigned long current_time = micros();
        float dt = (current_time - last_time) / 1000000.0f;
        last_time = current_time;
        const float cutoff = 2.0f;
        float RC = 1.0f / (2.0f * M_PI * cutoff);
        float alpha = dt / (RC + dt);
        filteredPitch += alpha * (pitchDeg - filteredPitch);
        filteredRoll  += alpha * (rollDeg - filteredRoll);
        tx_estring_value.clear();
        tx_estring_value.append("P:");
        tx_estring_value.append(filteredPitch);
        tx_estring_value.append(", R:");
        tx_estring_value.append(filteredRoll);
        tx_characteristic_string.writeValue(tx_estring_value.c_str());
        break;
      }
    case GET_FROM_GYROSCOPE:
      {
        static unsigned long last_time = micros();
        static float gyro_pitch = 0.0f, gyro_roll = 0.0f, gyro_yaw = 0.0f;
        static float comp_pitch = 0.0f, comp_roll = 0.0f;
        unsigned long current_time = micros();
        float dt = (current_time - last_time) / 1.e6;
        last_time = current_time;
        if (myICM.dataReady()) {
          myICM.getAGMT();
          gyro_pitch += myICM.gyrX() * dt;
          gyro_roll  += myICM.gyrY() * dt;
          gyro_yaw   += myICM.gyrZ() * dt;
          float pitch_acc = atan2(myICM.accX(), myICM.accZ()) * 180.0f / M_PI;
          float roll_acc  = atan2(myICM.accY(), myICM.accZ()) * 180.0f / M_PI;
          static float filteredPitch = pitch_acc;
          static float filteredRoll  = roll_acc;
          const float cutoff = 2.0f;
          float RC = 1.0f / (2.0f * M_PI * cutoff);
          float alpha_lp = dt / (RC + dt);
          filteredPitch += alpha_lp * (pitch_acc - filteredPitch);
          filteredRoll  += alpha_lp * (roll_acc - filteredRoll);
          const float alpha_g = 0.8f;
          comp_pitch = gyro_pitch * (1.0f - alpha_g) + filteredPitch * alpha_g;
          comp_roll  = gyro_roll  * (1.0f - alpha_g) + filteredRoll  * alpha_g;
          tx_estring_value.clear();
          tx_estring_value.append("Gyro Pitch: ");
          tx_estring_value.append(gyro_pitch);
          tx_estring_value.append("; Gyro Roll: ");
          tx_estring_value.append(gyro_roll);
          tx_estring_value.append("; Gyro Yaw: ");
          tx_estring_value.append(gyro_yaw);
          tx_estring_value.append("; Acc Pitch: ");
          tx_estring_value.append(pitch_acc);
          tx_estring_value.append("; Acc Roll: ");
          tx_estring_value.append(roll_acc);
          tx_estring_value.append("; CompPitch: ");
          tx_estring_value.append(comp_pitch);
          tx_estring_value.append("; CompRoll: ");
          tx_estring_value.append(comp_roll);
          tx_characteristic_string.writeValue(tx_estring_value.c_str());
          break;
        }
      }
    case STORE_DUAL_TOF_IMU_DATA:
      {
        dataSampleIndex = 0;
        memset(tof1Distances, 0, sizeof(tof1Distances));
        memset(tof2Distances, 0, sizeof(tof2Distances));
        memset(tofTimes, 0, sizeof(tofTimes));
        sampleIndex = 0;
        memset(timeStamps, 0, sizeof(timeStamps));
        memset(pitchArray, 0, sizeof(pitchArray));
        memset(rollArray, 0, sizeof(rollArray));
        recordingAll = true;
        recordAllStartTime = millis();
        Serial.println("Starting 5s ToF+IMU data recording...");
        break;
      }
    case SEND_DUAL_TOF_IMU_DATA:
      {
        Serial.println("Sending recorded ToF+IMU data...");
        for (int i = 0; i < dataSampleIndex; i++) {
          tx_estring_value.clear();
          tx_estring_value.append("TT:");
          tx_estring_value.append((int)tofTimes[i]);
          tx_estring_value.append("|T1:");
          tx_estring_value.append(tof1Distances[i]);
          tx_estring_value.append("|T2:");
          tx_estring_value.append(tof2Distances[i]);
          tx_characteristic_string.writeValue(tx_estring_value.c_str());
          delay(1);
        }
        for (int i = 0; i < sampleIndex; i++) {
          tx_estring_value.clear();
          tx_estring_value.append("T:");
          tx_estring_value.append((int)timeStamps[i]);
          tx_estring_value.append("|P:");
          tx_estring_value.append(pitchArray[i]);
          tx_estring_value.append("|R:");
          tx_estring_value.append(rollArray[i]);
          tx_characteristic_string.writeValue(tx_estring_value.c_str());
          delay(1);
        }
        Serial.println("Done sending ToF+IMU data.");
        break;
      }
    case SET_PID_GAINS:
      {
        Serial.println("Received SET_PID_GAINS command");
        float newKp, newKi, newKd, newSetpoint;
        bool success = robot_cmd.get_next_value(newKp);
        if (!success) { Serial.println("ERROR: Failed to get Kp"); return; }
        success = robot_cmd.get_next_value(newKi);
        if (!success) { Serial.println("ERROR: Failed to get Ki"); return; }
        success = robot_cmd.get_next_value(newKd);
        if (!success) { Serial.println("ERROR: Failed to get Kd"); return; }
        success = robot_cmd.get_next_value(newSetpoint);
        if (!success) { Serial.println("ERROR: Failed to get PID setpoint"); return; }
        Kp = newKp;
        Ki = newKi;
        Kd = newKd;
        pidSetpoint = newSetpoint;
        Serial.print("Updated PID Gains -> Kp: ");
        Serial.print(Kp);
        Serial.print(" | Ki: ");
        Serial.print(Ki);
        Serial.print(" | Kd: ");
        Serial.print(Kd);
        Serial.print(" | Setpoint: ");
        Serial.println(pidSetpoint);
        EString ack;
        ack.clear();
        ack.append("PID Gains Updated: Kp=");
        ack.append(Kp);
        ack.append(", Ki=");
        ack.append(Ki);
        ack.append(", Kd=");
        ack.append(Kd);
        ack.append(", Setpoint=");
        ack.append(pidSetpoint);
        tx_characteristic_string.writeValue(ack.c_str());
        break;
      }
    case START_PID_CONTROL:
      {
        Serial.println("Start PID Control Command");
        int newDuration = 0;
        bool success = robot_cmd.get_next_value(newDuration);
        if (success && newDuration > 0) {
          pidTestDuration = newDuration;
        }
        pidDataIndex = 0;
        memset(pidTimeArray, 0, sizeof(pidTimeArray));
        memset(pidErrorArray, 0, sizeof(pidErrorArray));
        memset(pidOutputArray, 0, sizeof(pidOutputArray));
        memset(pidMotorPWMArray, 0, sizeof(pidMotorPWMArray));
        memset(pidRawDistArray, 0, sizeof(pidRawDistArray));
        memset(pidExtrapolatedDistArray, 0, sizeof(pidExtrapolatedDistArray));
        pidActive = true;
        pidStartTime = millis();
        EString startAck;
        startAck.clear();
        startAck.append("Starting PID control for ");
        startAck.append(newDuration);
        startAck.append(" ms");
        tx_characteristic_string.writeValue(startAck.c_str());
        break;
      }
    case GET_PID_DATA:
      {
        Serial.println("Sending PID+ToF data...");
        for (int i = 0; i < pidDataIndex; i++) {
          EString dataStr;
          dataStr.clear();
          // Format: "T:<time>|E:<error>|U:<output>|R:<raw>|X:<extrapolated>|M:<motorPWM>"
          dataStr.append("T:");
          dataStr.append(pidTimeArray[i]);
          dataStr.append("|E:");
          dataStr.append(pidErrorArray[i]);
          dataStr.append("|U:");
          dataStr.append(pidOutputArray[i]);
          dataStr.append("|R:");
          dataStr.append(pidRawDistArray[i]);
          dataStr.append("|X:");
          dataStr.append(pidExtrapolatedDistArray[i]);
          dataStr.append("|M:");
          dataStr.append(pidMotorPWMArray[i]);
          tx_characteristic_string.writeValue(dataStr.c_str());
          delay(2);
        }
        Serial.println("PID+ToF data sent.");
        break;
      }
    default:
      Serial.print("Invalid Command Type: ");
      Serial.println(cmd_type);
      break;
  }
}

/****************************************************************
 * Setup
 ***************************************************************/
void setup() {
  Serial.begin(115200);
  BLE.begin();
  BLE.setDeviceName("Artemis BLE");
  BLE.setLocalName("Artemis BLE");
  BLE.setAdvertisedService(testService);
  testService.addCharacteristic(tx_characteristic_float);
  testService.addCharacteristic(tx_characteristic_string);
  testService.addCharacteristic(rx_characteristic_string);
  BLE.addService(testService);
  tx_characteristic_float.writeValue(0.0);
  Serial.print("Advertising BLE with MAC: ");
  Serial.println(BLE.address());
  BLE.advertise();
  while (!SERIAL_PORT) { };
  WIRE_PORT.begin();
  WIRE_PORT.setClock(400000);
  bool initialized = false;
  while (!initialized) {
    myICM.begin(WIRE_PORT, AD0_VAL);
    SERIAL_PORT.print(F("IMU init returned: "));
    SERIAL_PORT.println(myICM.statusString());
    if (myICM.status != ICM_20948_Stat_Ok) {
      SERIAL_PORT.println("Trying again...");
      delay(500);
    } else {
      initialized = true;
    }
  }
  Serial.println("IMU sensor initialized.");
  pinMode(SHUTDOWN_PIN, OUTPUT);
  digitalWrite(SHUTDOWN_PIN, LOW);
  distanceSensor1.setI2CAddress(0xf5);
  digitalWrite(SHUTDOWN_PIN, HIGH);
  if (distanceSensor1.begin() != 0) {
    Serial.println("Sensor1 failed to begin. Freezing...");
    while (1) { delay(10); }
  }
  Serial.println("Sensor1 online!");
  if (distanceSensor2.begin() != 0) {
    Serial.println("Sensor2 failed to begin. Freezing...");
    while (1) { delay(10); }
  }
  Serial.println("Sensor2 online!");
  distanceSensor1.setDistanceModeShort();
  distanceSensor2.setDistanceModeShort();
  distanceSensor1.startRanging();
  distanceSensor2.startRanging();
  pinMode(LED_BUILTIN, OUTPUT);
  for (int i = 0; i < 3; i++) {
    digitalWrite(LED_BUILTIN, HIGH);
    delay(200);
    digitalWrite(LED_BUILTIN, LOW);
    delay(200);
  }
}

/****************************************************************
 * Main Loop
 ***************************************************************/
void write_data() {
  currentMillis = millis();
  if (currentMillis - previousMillis > interval) {
    tx_float_value = tx_float_value + 0.5;
    tx_characteristic_float.writeValue(tx_float_value);
    if (tx_float_value > 10000) { tx_float_value = 0; }
    previousMillis = currentMillis;
  }
}

void read_data() {
  if (rx_characteristic_string.written()) {
    handle_command();
  }
}

void loop() {
  BLEDevice central = BLE.central();
  if (central) {
    Serial.print("Connected to: ");
    Serial.println(central.address());
    while (central.connected()) {
      write_data();
      read_data();
      if (pidActive) {
        unsigned long now = millis();
        unsigned long elapsed = now - pidStartTime;
        if (elapsed >= pidTestDuration) {
          pidActive = false;
          stopMotors();
          Serial.println("PID test finished by time limit.");
        } else {
          runPIDIteration();
        }
      }
    }
    pidActive = false;
    stopMotors();
    Serial.println("Disconnected. Motors stopped.");
  }
}
