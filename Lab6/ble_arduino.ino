#include "BLECStringCharacteristic.h" 
#include "EString.h"
#include "RobotCommand.h"
#include <ArduinoBLE.h>

// ========== BLE UUIDs ==========
#define BLE_UUID_TEST_SERVICE "ae3527ce-10d8-45c0-9f82-2158e073efba"
#define BLE_UUID_RX_STRING    "9750f60b-9c9c-4158-b620-02ec9521cd99"
#define BLE_UUID_TX_FLOAT     "27616294-3063-4ecc-b60b-3470ddef2938"
#define BLE_UUID_TX_STRING    "f235a225-6735-4d73-94cb-ee5dfce9ba83"

// Create BLE service and characteristics
BLEService testService(BLE_UUID_TEST_SERVICE);
BLECStringCharacteristic rx_characteristic_string(BLE_UUID_RX_STRING, BLEWrite, MAX_MSG_SIZE);
BLEFloatCharacteristic   tx_characteristic_float(BLE_UUID_TX_FLOAT, BLERead | BLENotify);
BLECStringCharacteristic tx_characteristic_string(BLE_UUID_TX_STRING, BLERead | BLENotify, MAX_MSG_SIZE);

// Notification-related globals
static double lastSendTime = 0;
static const double NOTIF_INTERVAL = 10;  // ms between sends

// Robot/BLE command parsing
RobotCommand robot_cmd(":|");
// TX
EString tx_estring_value;
float   tx_float_value = 0.0;

// General timing
long interval = 500;
static long previousMillis = 0;
unsigned long currentMillis = 0;

// ========== IMU / DMP Libraries & Globals ==========
#include "ICM_20948.h"
#define SERIAL_PORT Serial
#define WIRE_PORT  Wire
#define AD0_VAL    1

ICM_20948_I2C myICM;
bool dmpReady = false;

// Variables for storing quaternion->yaw results
static float lastYaw = 0.0f;  // keep track of last known yaw

// ========== Orientation PID Globals (Lab 6) ==========
float orientKp = 0.0f;
float orientKi = 0.0f;
float orientKd = 0.0f;
float orientSetpoint = 0.0f;    // desired yaw angle in degrees

bool orientPIDActive = false;
unsigned long orientStartTime = 0;
unsigned long orientTestDuration = 5000; // default 5s

static const int MAX_ORIENT_SAMPLES = 1000;
int   orientTimeArray[MAX_ORIENT_SAMPLES];
float orientYawArray[MAX_ORIENT_SAMPLES];
float orientErrorArray[MAX_ORIENT_SAMPLES];
float orientOutputArray[MAX_ORIENT_SAMPLES];
int   orientMotorCmdArray[MAX_ORIENT_SAMPLES];
int   orientDataIndex = 0;

// ========== ToF Libraries & Globals ==========
#include "SparkFun_VL53L1X.h"
#define SHUTDOWN_PIN 8
#define INTERRUPT_PIN 3

SFEVL53L1X distanceSensor1;
SFEVL53L1X distanceSensor2(Wire, SHUTDOWN_PIN, INTERRUPT_PIN);

// Arrays for storing distance data
static const int MAX_SAMPLES = 2000;
static int   tof1Distances[MAX_SAMPLES];
static int   tof2Distances[MAX_SAMPLES];
static unsigned long tofTimes[MAX_SAMPLES];
int  dataSampleIndex = 0;
bool recordingAll = false;
unsigned long recordAllStartTime = 0;

// IMU data arrays (for dual ToF + IMU logging)
unsigned long timeStamps[MAX_SAMPLES];
float pitchArray[MAX_SAMPLES];
float rollArray[MAX_SAMPLES];
int   sampleIndex = 0;
bool  recordingActive = false;
unsigned long recordStartTime = 0;
unsigned long currentTime = 0;

// Motor pins
const int MOTOR_BLUE_FWD = 6;
const int MOTOR_BLUE_REV = 7;
const int MOTOR_OTHER_FWD = 13;
const int MOTOR_OTHER_REV = 14;

// Motor calibration
const int MAX_PWM_OTHER = 65;                      
const int MAX_PWM_BLUE  = (int)(MAX_PWM_OTHER * 34.0 / 40.0);

// ===== PID and Extrapolation Globals (Lab 5 distance) =====

// Gains
float Kp = 0.0f;
float Ki = 0.0f;
float Kd = 0.0f;

// For logging PID
static const int MAX_PID_SAMPLES = 1000;
int   pidTimeArray[MAX_PID_SAMPLES];
float pidErrorArray[MAX_PID_SAMPLES];
float pidOutputArray[MAX_PID_SAMPLES];
float pidRawDistArray[MAX_PID_SAMPLES];
float pidExtrapolatedDistArray[MAX_PID_SAMPLES];
int   pidMotorPWMArray[MAX_PID_SAMPLES];

int   pidDataIndex = 0;
bool  pidActive = false;
unsigned long pidTestDuration = 5000; 
unsigned long pidStartTime = 0;
float pidSetpoint = 20.0;

// Extrapolation helpers
float distanceSensor1_cur = 0.0f, distanceSensor1_prev = 0.0f;
unsigned long timeSensor1_cur = 0, timeSensor1_prev = 0;
bool firstReading = true;

float lastRawDistance          = 0.0f;
float lastExtrapolatedDistance = 0.0f;

// Anti-windup
#define MAX_INTEGRAL 200.0f
bool antiWindupEnabled = true;

// Safety
bool bleConnected = false;

// ========== Command Types ==========
enum CommandTypes {
  GET_PITCH_ROLL = 0,
  GET_FROM_GYROSCOPE,
  STORE_IMU_DATA,
  SEND_IMU_DATA,
  TOF_DATA,
  STORE_DUAL_TOF_IMU_DATA,
  SEND_DUAL_TOF_IMU_DATA,
  SET_PID_GAINS,
  START_PID_CONTROL,
  GET_PID_DATA,

  // --- New for Orientation Control (Lab 6) ---
  SET_ORIENTATION_GAINS,
  START_ORIENTATION_CONTROL,
  GET_ORIENTATION_DATA
};

// ========== Helper Functions ==========
void stopMotors() {
  analogWrite(MOTOR_BLUE_FWD, 0);
  analogWrite(MOTOR_BLUE_REV, 0);
  analogWrite(MOTOR_OTHER_FWD, 0);
  analogWrite(MOTOR_OTHER_REV, 0);
}

int mapPIDToMotor(float pidVal) {
  if (pidVal > 255.0f)      pidVal = 255.0f;
  else if (pidVal < -255.0f) pidVal = -255.0f;
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
 * Extrapolation for distance sensor
 ***************************************************************/
float getDistanceSensor1Extrapolated() {
  bool newData = distanceSensor1.checkForDataReady();
  if (newData) {
    distanceSensor1_prev = distanceSensor1_cur;
    timeSensor1_prev     = timeSensor1_cur;
    int dist = distanceSensor1.getDistance();
    distanceSensor1.clearInterrupt();
    if (dist > 0) {
      distanceSensor1_cur = (float)dist;
      timeSensor1_cur     = millis();
    }
    if (firstReading) {
      distanceSensor1_prev = distanceSensor1_cur;
      timeSensor1_prev     = timeSensor1_cur;
      firstReading = false;
    }
    // For new data, raw and extrapolated are the same
    lastRawDistance          = distanceSensor1_cur;
    lastExtrapolatedDistance = distanceSensor1_cur;
    return distanceSensor1_cur;
  } else {
    // No new data -> extrapolate
    lastRawDistance = distanceSensor1_cur;
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
 * PID for ToF distance (Lab 5)
 ***************************************************************/
void runPIDIteration() {
  static unsigned long lastPIDTime = millis();
  unsigned long now = millis();
  float dt = (now - lastPIDTime) / 1000.0f;
  lastPIDTime = now;

  // get distance + extrapolation
  float extrapolatedDistance = getDistanceSensor1Extrapolated();
  float rawDistance = lastRawDistance; // last sensor reading
  float error = pidSetpoint - extrapolatedDistance;

  // Integrator
  static float integralTerm = 0.0f;
  integralTerm += error * dt;

  // Anti-windup
  if (antiWindupEnabled) {
    if (integralTerm > MAX_INTEGRAL)  integralTerm = MAX_INTEGRAL;
    if (integralTerm < -MAX_INTEGRAL) integralTerm = -MAX_INTEGRAL;
  }

  // Derivative
  static float previousError = 0.0f;
  float derivativeTerm = (error - previousError) / dt;
  previousError = error;

  // Combine
  float output = Kp * error + Ki * integralTerm + Kd * derivativeTerm;
  int motorCmd = mapPIDToMotor(output);
  setMotorSpeed(motorCmd);

  // Logging
  if (pidDataIndex < MAX_PID_SAMPLES) {
    pidTimeArray[pidDataIndex]        = (int)now;
    pidErrorArray[pidDataIndex]       = error;
    pidOutputArray[pidDataIndex]      = output;
    pidMotorPWMArray[pidDataIndex]    = motorCmd;
    pidRawDistArray[pidDataIndex]     = rawDistance;
    pidExtrapolatedDistArray[pidDataIndex] = extrapolatedDistance;
    pidDataIndex++;
  }
}

/****************************************************************
 * Get Yaw from DMP (using GAME_ROTATION_VECTOR -> Quat6)
 ***************************************************************/
float getCurrentYawFromDMP() {
  icm_20948_DMP_data_t dmpData;
  myICM.readDMPdataFromFIFO(&dmpData);
  // If no new data or error
  if ((myICM.status != ICM_20948_Stat_Ok) &&
      (myICM.status != ICM_20948_Stat_FIFOMoreDataAvail)) {
    return lastYaw;
  }
  // Check if we got Quat6 data
  if ((dmpData.header & DMP_header_bitmap_Quat6) == 0) {
    return lastYaw;
  }
  // Extract Q1,Q2,Q3 -> scale to +/- 1
  double q1 = ((double) dmpData.Quat6.Data.Q1) / 1073741824.0; 
  double q2 = ((double) dmpData.Quat6.Data.Q2) / 1073741824.0;
  double q3 = ((double) dmpData.Quat6.Data.Q3) / 1073741824.0;
  // Compute Q0
  double qw = sqrt(1.0 - min((q1*q1 + q2*q2 + q3*q3), 1.0));
  // Convert to yaw (Z-axis rotation)
  double siny = 2.0 * (qw * q3 + q1 * q2);
  double cosy = 1.0 - 2.0 * (q2*q2 + q3*q3);
  float yaw_deg = (float)(atan2(siny, cosy) * 180.0 / M_PI);
  lastYaw = yaw_deg;
  return yaw_deg;
}

/****************************************************************
 * Orientation (Yaw) PID (Lab 6)
 * Uses DMP for Yaw, derivative from gyroZ to avoid derivative kick
 ***************************************************************/
void runOrientationPIDIteration() {
  static unsigned long lastTime = millis();
  unsigned long now = millis();
  float dt = (now - lastTime) / 1000.0f;
  lastTime = now;
  // 1) Current yaw from DMP
  float currentYaw = getCurrentYawFromDMP(); // [-180..180], approx
  // 2) error
  float error = orientSetpoint - currentYaw;
  // handle wrap-around if desired:
  if (error > 180.0f)   error -= 360.0f;
  if (error < -180.0f)  error += 360.0f;
  // 3) Integrator
  static float integralTerm = 0.0f;
  integralTerm += error * dt;
  // clamp integrator
  if (integralTerm > MAX_INTEGRAL)  integralTerm = MAX_INTEGRAL;
  if (integralTerm < -MAX_INTEGRAL) integralTerm = -MAX_INTEGRAL;
  // 4) Derivative from gyro
  static float filteredGyroZ = 0.0f;
  float alpha = 0.2f; // low-pass
  float rawGyroZ = myICM.gyrZ(); // deg/sec
  filteredGyroZ = (1 - alpha)*filteredGyroZ + alpha*rawGyroZ;
    // derivative(e) = 0 - d(theta)/dt if setpoint is constant
  float derivativeTerm = -filteredGyroZ; 
  // 5) Combine
  float output = orientKp * error + orientKi * integralTerm + orientKd * derivativeTerm;
  // 6) Motor command: spin in place
  int motorCmd = (int)output;
  if (motorCmd > 150)  motorCmd = 150;
  if (motorCmd < -150) motorCmd = -150;
  if (motorCmd > 0) {
    analogWrite(MOTOR_BLUE_FWD, motorCmd);
    analogWrite(MOTOR_BLUE_REV, 0);
    analogWrite(MOTOR_OTHER_FWD, 0);
    analogWrite(MOTOR_OTHER_REV, motorCmd);
  } else if (motorCmd < 0) {
    int posCmd = abs(motorCmd);
    analogWrite(MOTOR_BLUE_FWD, 0);
    analogWrite(MOTOR_BLUE_REV, posCmd);
    analogWrite(MOTOR_OTHER_FWD, posCmd);
    analogWrite(MOTOR_OTHER_REV, 0);
  } else {
    stopMotors();
  }
  // 7) Log orientation data if desired
  if (orientDataIndex < MAX_ORIENT_SAMPLES) {
    orientTimeArray[orientDataIndex]    = now;
    orientYawArray[orientDataIndex]     = currentYaw;
    orientErrorArray[orientDataIndex]   = error;
    orientOutputArray[orientDataIndex]  = output;
    orientMotorCmdArray[orientDataIndex] = motorCmd;
    orientDataIndex++;
  }
}

/****************************************************************
 * Handle incoming BLE commands
 ***************************************************************/
void handle_command() {
  robot_cmd.set_cmd_string(rx_characteristic_string.value(), rx_characteristic_string.valueLength());
  bool success;
  int cmd_type = -1;
  success = robot_cmd.get_command_type(cmd_type);
  if (!success) { return; }

  switch (cmd_type) {

    case GET_PITCH_ROLL: {
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
      filteredRoll  += alpha * (rollDeg  - filteredRoll);

      tx_estring_value.clear();
      tx_estring_value.append("P:");
      tx_estring_value.append(filteredPitch);
      tx_estring_value.append(", R:");
      tx_estring_value.append(filteredRoll);
      tx_characteristic_string.writeValue(tx_estring_value.c_str());
      break;
    }

    case GET_FROM_GYROSCOPE: {
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
      }
      break;
    }

    case STORE_DUAL_TOF_IMU_DATA: {
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

    case SEND_DUAL_TOF_IMU_DATA: {
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

    case SET_PID_GAINS: {
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

    case START_PID_CONTROL: {
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

    case GET_PID_DATA: {
      Serial.println("Sending PID+ToF data...");
      for (int i = 0; i < pidDataIndex; i++) {
        EString dataStr;
        dataStr.clear();
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

    // ------------------------------
    // NEW Orientation PID Commands
    // ------------------------------
    case SET_ORIENTATION_GAINS: {
      float newKp, newKi, newKd, newSetpoint;
      if (!robot_cmd.get_next_value(newKp))       return;
      if (!robot_cmd.get_next_value(newKi))       return;
      if (!robot_cmd.get_next_value(newKd))       return;
      if (!robot_cmd.get_next_value(newSetpoint)) return;

      orientKp = newKp;
      orientKi = newKi;
      orientKd = newKd;
      orientSetpoint = newSetpoint;

      Serial.println("Orientation Gains Updated!");
      EString ack;
      ack.clear();
      ack.append("OK: orientKp=");
      ack.append(orientKp);
      ack.append(", orientKi=");
      ack.append(orientKi);
      ack.append(", orientKd=");
      ack.append(orientKd);
      ack.append(", setpoint=");
      ack.append(orientSetpoint);
      tx_characteristic_string.writeValue(ack.c_str());
      break;
    }

    case START_ORIENTATION_CONTROL: {
      Serial.println("Orientation PID Control Start");
      int newDuration = 0;
      if (robot_cmd.get_next_value(newDuration) && newDuration > 0) {
        orientTestDuration = newDuration;
      }
      // Clear orientation logs
      orientDataIndex = 0;
      memset(orientTimeArray, 0, sizeof(orientTimeArray));
      memset(orientYawArray, 0, sizeof(orientYawArray));
      memset(orientErrorArray, 0, sizeof(orientErrorArray));
      memset(orientOutputArray, 0, sizeof(orientOutputArray));
      memset(orientMotorCmdArray, 0, sizeof(orientMotorCmdArray));

      orientPIDActive = true;
      orientStartTime = millis();

      EString startAck;
      startAck.clear();
      startAck.append("Orientation PID for ");
      startAck.append(newDuration);
      startAck.append(" ms");
      tx_characteristic_string.writeValue(startAck.c_str());
      break;
    }

    case GET_ORIENTATION_DATA: {
      Serial.println("Sending orientation data...");
      for (int i = 0; i < orientDataIndex; i++) {
        EString dataStr;
        dataStr.clear();
        dataStr.append("T:");
        dataStr.append(orientTimeArray[i]);
        dataStr.append("|Y:");
        dataStr.append(orientYawArray[i]);
        dataStr.append("|E:");
        dataStr.append(orientErrorArray[i]);
        dataStr.append("|U:");
        dataStr.append(orientOutputArray[i]);
        dataStr.append("|M:");
        dataStr.append(orientMotorCmdArray[i]);
        tx_characteristic_string.writeValue(dataStr.c_str());
        delay(2);
      }
      Serial.println("Orientation data sent.");
      break;
    }

    default:
      Serial.print("Invalid Command Type: ");
      Serial.println(cmd_type);
      break;
  }
}

// ========== Setup ==========
void setup() {
  Serial.begin(115200);

  // BLE init
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

  // Wait for serial
  while (!SERIAL_PORT) { }

  // I2C
  WIRE_PORT.begin();
  WIRE_PORT.setClock(400000);

  // ========== IMU init (basic) ==========
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

  // ========== DMP Configuration ==========
  bool success = true;
  success &= (myICM.initializeDMP() == ICM_20948_Stat_Ok);
  success &= (myICM.enableDMPSensor(INV_ICM20948_SENSOR_GAME_ROTATION_VECTOR) == ICM_20948_Stat_Ok);
  // Set Quat6 ODR to max
  success &= (myICM.setDMPODRrate(DMP_ODR_Reg_Quat6, 0) == ICM_20948_Stat_Ok);
  success &= (myICM.enableFIFO() == ICM_20948_Stat_Ok);
  success &= (myICM.enableDMP() == ICM_20948_Stat_Ok);
  success &= (myICM.resetDMP() == ICM_20948_Stat_Ok);
  success &= (myICM.resetFIFO() == ICM_20948_Stat_Ok);

  if (!success) {
    Serial.println("DMP init failed! Check configs!");
  } else {
    Serial.println("DMP initialized successfully (GAME_ROTATION_VECTOR).");
  }

  // ========== ToF init ==========
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

// ========== Main Loop ==========
void write_data() {
  currentMillis = millis();
  if (currentMillis - previousMillis > interval) {
    tx_float_value = tx_float_value + 0.5;
    tx_characteristic_float.writeValue(tx_float_value);
    if (tx_float_value > 10000) {
      tx_float_value = 0;
    }
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

      // Distance PID (Lab 5) 
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

      // Orientation PID (Lab 6)
      if (orientPIDActive) {
        unsigned long now = millis();
        if ((now - orientStartTime) >= orientTestDuration) {
          orientPIDActive = false;
          stopMotors();
          Serial.println("Orientation test finished by time limit.");
        } else {
          runOrientationPIDIteration();
        }
      }
    }
    // Disconnected
    pidActive = false;
    orientPIDActive = false;
    stopMotors();
    Serial.println("Disconnected. Motors stopped.");
  }
}
