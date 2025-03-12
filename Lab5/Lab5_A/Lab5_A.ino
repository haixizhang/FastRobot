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
static const double NOTIF_INTERVAL = 10;  // milliseconds between sends, e.g. 10 ms
// RX
RobotCommand robot_cmd(":|");
// TX
EString tx_estring_value;
float tx_float_value = 0.0;

long interval = 500;
static long previousMillis = 0;
unsigned long currentMillis = 0;

// for lab 2
#include "ICM_20948.h"  // Click here to get the library: http://librarymanager/All#SparkFun_ICM_20948_IMU

//#define USE_SPI       // Uncomment this to use SPI
#define SERIAL_PORT Serial
#define SPI_PORT SPI    // Your desired SPI port.       Used only when "USE_SPI" is defined
#define CS_PIN 2        // Which pin you connect CS to. Used only when "USE_SPI" is defined
#define WIRE_PORT Wire  // Your desired Wire port.      Used when "USE_SPI" is not defined
// The value of the last bit of the I2C address.
// On the SparkFun 9DoF IMU breakout the default is 1, and when the ADR jumper is closed the value becomes 0
#define AD0_VAL 1
#ifdef USE_SPI
ICM_20948_SPI myICM;  // If using SPI create an ICM_20948_SPI object
#else
ICM_20948_I2C myICM;  // Otherwise create an ICM_20948_I2C object
#endif
// Global arrays to store IMU data:
bool recordingActive = false;
unsigned long recordStartTime = 0;
unsigned long currentTime = 0;
const int MAX_SAMPLES = 2000;
unsigned long timeStamps[MAX_SAMPLES];
// Arrays to store complementary filtered pitch and roll
float pitchArray[MAX_SAMPLES], rollArray[MAX_SAMPLES];
int sampleIndex = 0;
int failed_attempt = 0;

// TOF Variables
#include <Wire.h>
#include "SparkFun_VL53L1X.h"
#define SHUTDOWN_PIN 8
#define INTERRUPT_PIN 3
SFEVL53L1X distanceSensor1;
SFEVL53L1X distanceSensor2(Wire, SHUTDOWN_PIN, INTERRUPT_PIN);
// Maximum number of samples to store
const int MAX_DATA_SAMPLES = 2000;
// For two ToF sensors:
static int tof1Distances[MAX_DATA_SAMPLES];
static int tof2Distances[MAX_DATA_SAMPLES];
static unsigned long tofTimes[MAX_DATA_SAMPLES];
// A separate index for these measurements
int dataSampleIndex = 0;
// A flag to know when we’re recording
bool recordingAll = false;
unsigned long recordAllStartTime = 0;

/***** Global Variables for Motors *****/
// Define motor driver pins
const int MOTOR_BLUE_FWD = 6;    // Left (blue) motor forward
const int MOTOR_BLUE_REV = 7;    // Left (blue) motor reverse
const int MOTOR_OTHER_FWD = 13;  // Right (other) motor forward
const int MOTOR_OTHER_REV = 14;  // Right (other) motor reverse

// Calibration constants for PWM output
const int MAX_PWM_OTHER = 65;                                 // Right motor forward PWM (calibrated higher)
const int MAX_PWM_BLUE = (int)(MAX_PWM_OTHER * 34.0 / 40.0);  // Left motor forward PWM (calibrated)

/***** Global Variables for PID + ToF *****/

// PID gains
float Kp = 0.0f;
float Ki = 0.0f;
float Kd = 0.0f;

// Logging arrays
static const int MAX_PID_SAMPLES = 1000;
int pidTimeArray[MAX_PID_SAMPLES];       // Timestamps
float pidErrorArray[MAX_PID_SAMPLES];    // Error (Setpoint - distance)
float pidOutputArray[MAX_PID_SAMPLES];   // Controller output
float pidTofDistArray[MAX_PID_SAMPLES];  // ToF distance (for debugging)
int pidMotorPWMArray[MAX_PID_SAMPLES];   // Motor PWM command (new logging array)

// Index to track how many samples we've stored
int pidDataIndex = 0;

// Control flags
bool pidActive = false;
unsigned long pidTestDuration = 5000;  // default 5 seconds
unsigned long pidStartTime = 0;

// The desired stopping distance (e.g. 304mm for 1ft)
float pidSetpoint = 20.0;

// For safety if BLE disconnects
bool bleConnected = false;

//////////// Global Variables ////////////

enum CommandTypes {
  GET_PITCH_ROLL,
  GET_FROM_GYROSCOPE,
  STORE_IMU_DATA,
  SEND_IMU_DATA,
  TOF_DATA,
  STORE_DUAL_TOF_IMU_DATA,
  SEND_DUAL_TOF_IMU_DATA,
  SET_PID_GAINS,      // <cmd_id> <Kp> <Ki> <Kd>|<setpoint>
  START_PID_CONTROL,  // <cmd_id> <duration_ms> (optional)
  GET_PID_DATA,       // <cmd_id> request to send logged data
};

// Stop the motors by writing 0 to all channels
void stopMotors() {
  analogWrite(MOTOR_BLUE_FWD, 0);
  analogWrite(MOTOR_BLUE_REV, 0);
  analogWrite(MOTOR_OTHER_FWD, 0);
  analogWrite(MOTOR_OTHER_REV, 0);
}

int mapPIDToMotor(float pidVal) {
  // Clamp the value to the range -255 to 255
  if (pidVal > 255.0f) {
    pidVal = 255.0f;
  } else if (pidVal < -255.0f) {
    pidVal = -255.0f;
  }
  return (int)pidVal;
}

// Set motor speeds based on a unified PID output value
void setMotorSpeed(int motorCmd) {
  // Calculate the absolute value of the command (clamped to 100)
  int absCmd = abs(motorCmd);
  if (absCmd > 100) absCmd = 100;

  // Scale the PID output to the calibrated PWM range for each motor:
  int pwmRight = absCmd;                      // Right motor (other)
  int pwmLeft = (int)(absCmd * 32.0 / 40.0);  // Left motor (blue)

  // Decide the direction based on the sign of motorCmd
  if (motorCmd > 0) {
    // Forward: activate forward pins, ensure reverse pins are zero
    analogWrite(MOTOR_BLUE_FWD, pwmLeft);
    analogWrite(MOTOR_BLUE_REV, 0);
    analogWrite(MOTOR_OTHER_FWD, pwmRight);
    analogWrite(MOTOR_OTHER_REV, 0);
  } else if (motorCmd < 0) {
    // Reverse: activate reverse pins, ensure forward pins are zero
    analogWrite(MOTOR_BLUE_FWD, 0);
    analogWrite(MOTOR_BLUE_REV, pwmLeft);
    analogWrite(MOTOR_OTHER_FWD, 0);
    analogWrite(MOTOR_OTHER_REV, pwmRight);
  } else {
    // Zero command: stop motors
    stopMotors();
  }
}

float readDistanceSensor1() {
  // These lines assume your sensor is called `distanceSensor1`
  // and is already started in setup(), e.g. distanceSensor1.startRanging();
  static float lastDistance = 0.0f;  // retain last known good reading

  // Non-blocking check to see if new data is ready
  if (distanceSensor1.checkForDataReady()) {
    int dist = distanceSensor1.getDistance();  // returns distance in mm
    distanceSensor1.clearInterrupt();

    if (dist > 0) {
      lastDistance = (float)dist;  // update last known distance
    }
  }
  return lastDistance;  // return either the new reading or the last known
}

void runPIDIteration() {
  static unsigned long lastPIDTime = millis();
  unsigned long now = millis();
  float dt = (now - lastPIDTime) / 1000.0f;  // dt in seconds
  lastPIDTime = now;

  // --- 1) Get the current distance from the ToF sensor ---
  float currentDistance = readDistanceSensor1();
  // --- 2) Compute error for PID ---
  float error = pidSetpoint - currentDistance;
  // --- 3)  Keep track of integral and derivative ---
  static float integralTerm = 0.0f;
  integralTerm += error * dt;
  static float previousError = 0.0f;
  float derivativeTerm = (error - previousError) / dt;
  previousError = error;
  // --- 4) Calculate PID output ---
  float output = Kp * error + Ki * integralTerm + Kd * derivativeTerm;
  // --- 5) Convert output to motor commands ---
  int motorCmd = mapPIDToMotor(output);
  setMotorSpeed(motorCmd);  // Command the motors
  // --- 6) Log data (ToF distance, error, output, timestamp, and motor PWM) ---
  if (pidDataIndex < MAX_PID_SAMPLES) {
    pidTimeArray[pidDataIndex] = (int)now;
    pidErrorArray[pidDataIndex] = error;
    pidOutputArray[pidDataIndex] = output;
    pidTofDistArray[pidDataIndex] = currentDistance;
    pidMotorPWMArray[pidDataIndex] = motorCmd;  // Log the motor PWM command
    pidDataIndex++;
  }
}

void handle_command() {
  // Set the command string from the characteristic value
  robot_cmd.set_cmd_string(rx_characteristic_string.value(), rx_characteristic_string.valueLength());
  bool success;
  int cmd_type = -1;
  success = robot_cmd.get_command_type(cmd_type);
  // Check if the last tokenization was successful and return if failed
  if (!success) {
    return;
  }
  // Handle the command type accordingly
  switch (cmd_type) {
    case GET_PITCH_ROLL:
      {
        myICM.getAGMT();
        float ax = myICM.accX() / 1000.0f;
        float ay = myICM.accY() / 1000.0f;
        float az = myICM.accZ() / 1000.0f;
        float pitchDeg = atan2(ax, az) * 180.0f / M_PI;
        float rollDeg = atan2(ay, az) * 180.0f / M_PI;
        // Build the response string, for example: "P:<pitch>, R:<roll>"
        // ---- Implement the Lowpass Filter ----
        static float filteredPitch = pitchDeg;  // Initialize on first call.
        static float filteredRoll = rollDeg;
        // Compute the elapsed time (dt) in seconds.
        static unsigned long last_time = micros();
        unsigned long current_time = micros();
        float dt = (current_time - last_time) / 1000000.0f;  // dt in seconds
        last_time = current_time;
        // Define the cutoff frequency in Hz.
        const float cutoff = 2.0f;  // Adjust as needed.
        // Calculate RC and the filter coefficient alpha.
        float RC = 1.0f / (2.0f * M_PI * cutoff);
        float alpha = dt / (RC + dt);
        // Update the filtered values.
        filteredPitch = filteredPitch + alpha * (pitchDeg - filteredPitch);
        filteredRoll = filteredRoll + alpha * (rollDeg - filteredRoll);
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
        // Use static variables to retain values across calls.
        static unsigned long last_time = micros();
        static float gyro_pitch = 0.0f, gyro_roll = 0.0f, gyro_yaw = 0.0f;
        static float comp_pitch = 0.0f, comp_roll = 0.0f;
        // Use the same dt for gyro integration and lowpass filtering.
        unsigned long current_time = micros();
        float dt = (current_time - last_time) / 1.e6;
        last_time = current_time;
        if (myICM.dataReady()) {
          myICM.getAGMT();
          // Gyro integration (angles in degrees)
          gyro_pitch += myICM.gyrX() * dt;
          gyro_roll += myICM.gyrY() * dt;
          gyro_yaw += myICM.gyrZ() * dt;
          // Compute pitch and roll from accelerometer.
          float pitch_acc = atan2(myICM.accX(), myICM.accZ()) * 180.0f / M_PI;
          float roll_acc = atan2(myICM.accY(), myICM.accZ()) * 180.0f / M_PI;
          // --- Lowpass Filter on Accelerometer Angles ---
          static float filteredPitch = pitch_acc;
          static float filteredRoll = roll_acc;
          const float cutoff = 2.0f;  // cutoff frequency in Hz.
          float RC = 1.0f / (2.0f * M_PI * cutoff);
          float alpha_lp = dt / (RC + dt);  // Lowpass filter coefficient.
          filteredPitch = filteredPitch + alpha_lp * (pitch_acc - filteredPitch);
          filteredRoll = filteredRoll + alpha_lp * (roll_acc - filteredRoll);
          // --- Complementary Filter ---
          const float alpha_g = 0.8f;
          comp_pitch = gyro_pitch * (1.0f - alpha_g) + filteredPitch * alpha_g;
          comp_roll = gyro_roll * (1.0f - alpha_g) + filteredRoll * alpha_g;
          // --- Build and Send the Response ---
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
        // Reset ToF arrays
        dataSampleIndex = 0;
        memset(tof1Distances, 0, sizeof(tof1Distances));
        memset(tof2Distances, 0, sizeof(tof2Distances));
        memset(tofTimes, 0, sizeof(tofTimes));
        // Reset IMU arrays
        sampleIndex = 0;
        memset(timeStamps, 0, sizeof(timeStamps));
        memset(pitchArray, 0, sizeof(pitchArray));
        memset(rollArray, 0, sizeof(rollArray));
        // Set recording flag and record start time
        recordingAll = true;
        recordAllStartTime = millis();
        Serial.println("Starting 5s ToF+IMU data recording with complementary filtered pitch & roll.");
        break;
      }

    case SEND_DUAL_TOF_IMU_DATA:
      {
        Serial.println("Sending recorded ToF+IMU data...");
        // 1) Send ToF data
        for (int i = 0; i < dataSampleIndex; i++) {
          tx_estring_value.clear();
          // Format: "TT:<time>|T1:<distSensor1>|T2:<distSensor2>"
          tx_estring_value.append("TT:");
          tx_estring_value.append((int)tofTimes[i]);
          tx_estring_value.append("|T1:");
          tx_estring_value.append(tof1Distances[i]);
          tx_estring_value.append("|T2:");
          tx_estring_value.append(tof2Distances[i]);
          tx_characteristic_string.writeValue(tx_estring_value.c_str());
          delay(1);  // short delay to avoid flooding
        }
        // 2) Send IMU data as complementary filtered pitch and roll
        for (int i = 0; i < sampleIndex; i++) {
          tx_estring_value.clear();
          // Format: "T:<time>|P:<pitch>|R:<roll>"
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
        if (!success) {
          Serial.println("ERROR: Failed to get Kp");
          return;
        }

        success = robot_cmd.get_next_value(newKi);
        if (!success) {
          Serial.println("ERROR: Failed to get Ki");
          return;
        }

        success = robot_cmd.get_next_value(newKd);
        if (!success) {
          Serial.println("ERROR: Failed to get Kd");
          return;
        }

        success = robot_cmd.get_next_value(newSetpoint);
        if (!success) {
          Serial.println("ERROR: Failed to get PID setpoint");
          return;
        }

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

        // Send acknowledgement over BLE
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
        Serial.println("Start");
        // Optionally parse the test duration from the command
        int newDuration = 0;  // Use int instead of unsigned long
        bool success = robot_cmd.get_next_value(newDuration);
        if (success && newDuration > 0) {
          pidTestDuration = newDuration;  // pidTestDuration can remain unsigned long if needed
        }

        // Reset indexes for logging
        pidDataIndex = 0;
        memset(pidTimeArray, 0, sizeof(pidTimeArray));
        memset(pidErrorArray, 0, sizeof(pidErrorArray));
        memset(pidOutputArray, 0, sizeof(pidOutputArray));
        memset(pidTofDistArray, 0, sizeof(pidTofDistArray));
        memset(pidMotorPWMArray, 0, sizeof(pidMotorPWMArray));
        
        // Mark control as active
        pidActive = true;
        pidStartTime = millis();

        // Send an acknowledgement
        EString startAck;
        startAck.clear();
        startAck.append("Starting PID control for ");
        startAck.append(newDuration);  // newDuration is now int
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
          // Format: "T:<time>|E:<error>|U:<output>|D:<distance>|M:<motorPWM>"
          dataStr.append("T:");
          dataStr.append(pidTimeArray[i]);
          dataStr.append("|E:");
          dataStr.append(pidErrorArray[i]);
          dataStr.append("|U:");
          dataStr.append(pidOutputArray[i]);
          dataStr.append("|D:");
          dataStr.append(pidTofDistArray[i]);
          dataStr.append("|M:");
          dataStr.append(pidMotorPWMArray[i]);
          tx_characteristic_string.writeValue(dataStr.c_str());
          delay(2);  // short delay to avoid BLE overflow
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

void setup() {
  Serial.begin(115200);
  BLE.begin();

  // Set advertised local name and service
  BLE.setDeviceName("Artemis BLE");
  BLE.setLocalName("Artemis BLE");
  BLE.setAdvertisedService(testService);

  // Add BLE characteristics
  testService.addCharacteristic(tx_characteristic_float);
  testService.addCharacteristic(tx_characteristic_string);
  testService.addCharacteristic(rx_characteristic_string);

  // Add BLE service
  BLE.addService(testService);

  // Initial values for characteristics
  // Set initial values to prevent errors when reading for the first time on central devices
  tx_characteristic_float.writeValue(0.0);
  // Output MAC Address
  Serial.print("Advertising BLE with MAC: ");
  Serial.println(BLE.address());
  BLE.advertise();
  while (!SERIAL_PORT) {
  };

#ifdef USE_SPI
  SPI_PORT.begin();
#else
  WIRE_PORT.begin();
  WIRE_PORT.setClock(400000);
#endif
  //myICM.enableDebugging(); // Uncomment this line to enable helpful debug messages on Serial
  bool initialized = false;
  while (!initialized) {
#ifdef USE_SPI
    myICM.begin(CS_PIN, SPI_PORT);
#else
    myICM.begin(WIRE_PORT, AD0_VAL);
#endif
    SERIAL_PORT.print(F("Initialization of the IMU sensor returned: "));
    SERIAL_PORT.println(myICM.statusString());
    if (myICM.status != ICM_20948_Stat_Ok) {
      SERIAL_PORT.println("Trying again...");
      delay(500);
    } else {
      initialized = true;
    }
  }
  Serial.println("IMU sensor initialized.");

  // Initialize I2C
  Wire.begin();
  Wire.setClock(400000);  // 400 kHz I2C if desired

  pinMode(SHUTDOWN_PIN, OUTPUT);
  digitalWrite(SHUTDOWN_PIN, LOW);  // Disable sensor2 initially
  distanceSensor1.setI2CAddress(0xf5);
  // Re-enable sensor2
  digitalWrite(SHUTDOWN_PIN, HIGH);
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
  // Query if the characteristic value has been written by another BLE device
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
      // 1) BLE housekeeping
      write_data();
      read_data();

      // 2) If PID is active, run the control
      if (pidActive) {
        unsigned long now = millis();
        unsigned long elapsed = now - pidStartTime;

        // Hard stop if we exceed the test duration
        if (elapsed >= pidTestDuration) {
          pidActive = false;
          stopMotors();
          Serial.println("PID test finished by time limit.");
        } else {
          // Perform one PID iteration
          runPIDIteration();
        }
      }
    }

    // If we get here, BLE is disconnected
    pidActive = false;
    stopMotors();  // Safety
    Serial.println("Disconnected. Motors stopped.");
  }
}
