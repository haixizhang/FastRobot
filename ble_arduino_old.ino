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


// for TIME_LOOP
static bool time_loop_active = false;
static long last_time_send = 0;
static long TIME_LOOP_INTERVAL = 10;


// for Question 6
// Define maximum number of time stamps to store
const int MAX_TIME_STAMPS = 100;
// Global array to store time stamps
int time_stamps[MAX_TIME_STAMPS];
// for Question 7
float temp_stamps[MAX_TIME_STAMPS];


// for lab 2
#include "ICM_20948.h"  // Click here to get the library: http://librarymanager/All#SparkFun_ICM_20948_IMU


//#define USE_SPI       // Uncomment this to use SPI


#define SERIAL_PORT Serial


#define SPI_PORT SPI  // Your desired SPI port.       Used only when "USE_SPI" is defined
#define CS_PIN 2      // Which pin you connect CS to. Used only when "USE_SPI" is defined


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


// We'll reuse the existing IMU arrays: axArray, ayArray, etc.


// A separate index for these measurements
int dataSampleIndex = 0;


// A flag to know when we’re recording
bool recordingAll = false;
unsigned long recordAllStartTime = 0;


//////////// Global Variables ////////////


enum CommandTypes {
  PING,
  SEND_TWO_INTS,
  SEND_THREE_FLOATS,
  ECHO,
  DANCE,
  SET_VEL,
  GET_TIME_MILLIS,
  GET_TIME_STREAM,
  STORE_TIME_DATA,
  SEND_TIME_DATA,
  STORE_TEMP_DATA,
  GET_TEMP_READINGS,
  SEND_SHORT_REPLY,
  SEND_LONG_REPLY,
  HIGH_RATE_TEST,
  SIZED_REPLY,
  GET_PITCH_ROLL,
  GET_FROM_GYROSCOPE,
  STORE_IMU_DATA,
  SEND_IMU_DATA,
  TOF_DATA,
  STORE_DUAL_TOF_IMU_DATA,
  SEND_DUAL_TOF_IMU_DATA
};


void handle_command() {
  // Set the command string from the characteristic value
  robot_cmd.set_cmd_string(rx_characteristic_string.value(),
                           rx_characteristic_string.valueLength());


  bool success;
  int cmd_type = -1;


  // Get robot command type (an integer)
  /* NOTE: THIS SHOULD ALWAYS BE CALLED BEFORE get_next_value()
     * since it uses strtok internally (refer RobotCommand.h and
     * https://www.cplusplus.com/reference/cstring/strtok/)
     */
  success = robot_cmd.get_command_type(cmd_type);


  // Check if the last tokenization was successful and return if failed
  if (!success) {
    return;
  }


  // Handle the command type accordingly
  switch (cmd_type) {
    /*
         * Write "PONG" on the GATT characteristic BLE_UUID_TX_STRING
         */
    case PING:
      {
        tx_estring_value.clear();
        tx_estring_value.append("PONG");
        tx_characteristic_string.writeValue(tx_estring_value.c_str());
        Serial.print("Sent back: ");
        Serial.println(tx_estring_value.c_str());
      }
    /*
         * Extract two integers from the command string
         */
    case SEND_TWO_INTS:
      {
        int int_a, int_b;
        // Extract the next value from the command string as an integer
        success = robot_cmd.get_next_value(int_a);
        if (!success)
          return;
        // Extract the next value from the command string as an integer
        success = robot_cmd.get_next_value(int_b);
        if (!success)
          return;
        Serial.print("Two Integers: ");
        Serial.print(int_a);
        Serial.print(", ");
        Serial.println(int_b);
        break;
      }
    /*
         * Extract three floats from the command string
         */
    case SEND_THREE_FLOATS:
      {
        float ft_a, ft_b, ft_c;
        // Extract the next value from the command string as an integer
        success = robot_cmd.get_next_value(ft_a);
        if (!success)
          return;
        success = robot_cmd.get_next_value(ft_b);
        if (!success)
          return;
        success = robot_cmd.get_next_value(ft_c);
        if (!success)
          return;
        tx_estring_value.clear();
        tx_estring_value.append("Received: ");
        tx_estring_value.append(ft_a);
        tx_estring_value.append(", ");
        tx_estring_value.append(ft_b);
        tx_estring_value.append(", ");
        tx_estring_value.append(ft_c);
        tx_characteristic_string.writeValue(tx_estring_value.c_str());
        Serial.print("Sent back Floats:");
        Serial.println(tx_estring_value.c_str());
        break;
      }
    case ECHO:
      {
        char char_arr[MAX_MSG_SIZE];
        // Extract the next value from the command string as a character array
        success = robot_cmd.get_next_value(char_arr);
        if (!success)
          return;
        // Define prefix and postfix
        const char* prefix = "Robot says -> ";
        const char* postfix = " :)";
        // Create a new string with prefix and postfix
        tx_estring_value.clear();
        tx_estring_value.append(prefix);
        tx_estring_value.append(char_arr);
        tx_estring_value.append(postfix);
        // Write the modified string back to the BLE characteristic
        tx_characteristic_string.writeValue(tx_estring_value.c_str());
        // Log the operation
        Serial.print("Echoed back: ");
        Serial.println(tx_estring_value.c_str());
        break;
      }
    /*
         * DANCE
         */
    case DANCE:
      {
        Serial.println("Look Ma, I'm Dancin'!");
        break;
      }
    /*
         * SET_VEL
         */
    case SET_VEL:
      break;
      /*
         * The default case may not capture all types of invalid commands.
         * It is safer to validate the command string on the central device (in python)
         * before writing to the characteristic.
         */
    case GET_TIME_MILLIS:
      {
        tx_estring_value.clear();
        tx_estring_value.append("T:");
        tx_estring_value.append((int)millis());
        tx_characteristic_string.writeValue(tx_estring_value.c_str());
        Serial.print("Sent time: ");
        Serial.println(tx_estring_value.c_str());
        break;
      }
    case GET_TIME_STREAM:
      {
        Serial.println("Starting time streaming for 5 seconds...");
        // Clear out any leftover data from the EString
        tx_estring_value.clear();
        // Record the starting time
        unsigned long start_time = millis();
        unsigned long duration_ms = 5000;  // Stream for 5 seconds
        while (millis() - start_time < duration_ms) {
          // Build a string of the form "T:123456"
          tx_estring_value.clear();
          tx_estring_value.append("T:");
          tx_estring_value.append((int)millis());
          // Write to the TX_STRING characteristic
          tx_characteristic_string.writeValue(tx_estring_value.c_str());
          // Small delay so we don't overwhelm the BLE connection
          delay(5);
        }
        Serial.println("Finished streaming time.");
        break;
      }
    case STORE_TIME_DATA:
      {
        Serial.println("Starting time streaming for 5 seconds...");
        unsigned long start_time = millis();
        unsigned long duration_ms = 5000;  // Stream for 5 seconds
        unsigned long current_time = start_time;
        int i = 0;
        while (millis() - start_time < duration_ms) {
          current_time = millis();
          time_stamps[i] = (int)current_time;
          if (i++ >= MAX_TIME_STAMPS) { break; }
          delay(5);
        }
        Serial.println("Finished streaming time.");
        break;
      }


    case SEND_TIME_DATA:
      {
        Serial.println("Received SEND_TIME_DATA command. Sending stored timestamps...");
        for (int i = 0; i < MAX_TIME_STAMPS; i++) {
          // Edge Case: no empty array
          if (time_stamps[i] < 1) { break; }


          tx_estring_value.clear();
          tx_estring_value.append("T:");
          tx_estring_value.append(time_stamps[i]);
          tx_characteristic_string.writeValue(tx_estring_value.c_str());
        }
        Serial.print("Done sending ");
        break;
      }
    case STORE_TEMP_DATA:
      {
        unsigned long start_time = millis();
        unsigned long current_time = start_time;
        int i = 0;
        while (current_time - start_time < 5000) {
          current_time = millis();
          time_stamps[i] = (int)current_time;
          temp_stamps[i] = getTempDegC();
          // Prevent over-filling the storage array
          if (i++ >= MAX_TIME_STAMPS) break;
        }
        Serial.println("Finished streaming temp.");
        break;
      }
    case GET_TEMP_READINGS:
      {
        Serial.println("Sending time+temperature data...");
        for (int i = 0; i < MAX_TIME_STAMPS; i++) {
          tx_estring_value.clear();


          // Format: "T:<time>|<temperature>"
          tx_estring_value.append("T:");
          tx_estring_value.append(time_stamps[i]);
          tx_estring_value.append("|");
          tx_estring_value.append(temp_stamps[i]);
          tx_characteristic_string.writeValue(tx_estring_value.c_str());
          delay(5);
        }
        Serial.println("Done sending time+temp data.");
        break;
      }
    case SEND_SHORT_REPLY:
      {
        // 5-byte string reply "Hello"
        tx_estring_value.clear();
        tx_estring_value.append("Hello");  // 5 characters
        tx_characteristic_string.writeValue(tx_estring_value.c_str());
        break;
      }
    case SEND_LONG_REPLY:
      {
        // 120-byte string reply
        tx_estring_value.clear();
        for (int i = 0; i < 120; i++) {
          tx_estring_value.append("X");  // dummy character
        }
        tx_characteristic_string.writeValue(tx_estring_value.c_str());
        break;
      }
    case HIGH_RATE_TEST:
      {
        // We'll send 200 messages quickly.
        int NUM_MSGS = 200;
        Serial.println("Starting HIGH_RATE_TEST...");
        for (int i = 0; i < NUM_MSGS; i++) {
          tx_estring_value.clear();
          tx_estring_value.append("MSG:");
          tx_estring_value.append(i);
          tx_characteristic_string.writeValue(tx_estring_value.c_str());
          // Adjust this delay to control the rate of sending
          delay(0);
        }
        Serial.println("HIGH_RATE_TEST complete.");
        break;
      }
    case SIZED_REPLY:
      {
        int requested_length = 0;
        success = robot_cmd.get_next_value(requested_length);
        if (!success) {
          Serial.println("Error parsing requested length in SIZED_REPLY");
          return;
        }
        // Build a string with that many bytes
        tx_estring_value.clear();
        for (int i = 0; i < requested_length; i++) {
          tx_estring_value.append("X");
        }
        tx_characteristic_string.writeValue(tx_estring_value.c_str());
        Serial.print("Sending reply of length: ");
        Serial.println(requested_length);
        break;
      }
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
    case SEND_DUAL_TOF_IMU_DATA:{
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


    SERIAL_PORT.print(F("Initialization of the sensor returned: "));
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
  // Listen for connections
  BLEDevice central = BLE.central();


  // If a central is connected to the peripheral
  if (central) {
    Serial.print("Connected to: ");
    Serial.println(central.address());
    while (central.connected()) {
      // Send data
      write_data();
      // Read data
      read_data();
      if (recordingAll) {
        unsigned long now = millis();
        if ((now - recordAllStartTime) > 5000 ||                                // Past 5 seconds
            dataSampleIndex >= MAX_DATA_SAMPLES || sampleIndex >= MAX_SAMPLES)  // or arrays full
        {
          recordingAll = false;
          Serial.println("Stopped ToF+IMU recording.");
        } else {
          // ---- IMU Part  ----
          if (myICM.dataReady()) {
            myICM.getAGMT();
            // Use micros() to compute dt in seconds
            static unsigned long lastTimeIMU = micros();
            unsigned long currentTimeIMU = micros();
            float dt = (currentTimeIMU - lastTimeIMU) / 1000000.0f;  // dt in seconds
            lastTimeIMU = currentTimeIMU;

            // Gyro integration (angles in degrees)
            static float gyro_pitch = 0.0f, gyro_roll = 0.0f;
            gyro_pitch += myICM.gyrX() * dt;
            gyro_roll += myICM.gyrY() * dt;

            // Compute accelerometer-based angles
            float pitch_acc = atan2(myICM.accX(), myICM.accZ()) * 180.0f / M_PI;
            float roll_acc = atan2(myICM.accY(), myICM.accZ()) * 180.0f / M_PI;

            // Lowpass filter on accelerometer angles
            static float filteredPitch = pitch_acc, filteredRoll = roll_acc;
            const float cutoff = 2.0f;  // Hz
            float RC = 1.0f / (2.0f * M_PI * cutoff);
            float alpha_lp = dt / (RC + dt);
            filteredPitch = filteredPitch + alpha_lp * (pitch_acc - filteredPitch);
            filteredRoll = filteredRoll + alpha_lp * (roll_acc - filteredRoll);

            // Complementary filter: combine gyro integration and accelerometer data
            const float alpha_g = 0.8f;
            float comp_pitch = gyro_pitch * (1.0f - alpha_g) + filteredPitch * alpha_g;
            float comp_roll = gyro_roll * (1.0f - alpha_g) + filteredRoll * alpha_g;
            // Store the computed complementary filtered pitch and roll with timestamp
            timeStamps[sampleIndex] = now;
            pitchArray[sampleIndex] = comp_pitch;
            rollArray[sampleIndex] = comp_roll;
            sampleIndex++;
          }

          // ---- ToF Part (Non-blocking) ----
          if (distanceSensor1.checkForDataReady()) {
            tofTimes[dataSampleIndex] = now;  // store time
            tof1Distances[dataSampleIndex] = distanceSensor1.getDistance();
            distanceSensor1.clearInterrupt();
          }
          if (distanceSensor2.checkForDataReady()) {
            tof2Distances[dataSampleIndex] = distanceSensor2.getDistance();
            distanceSensor2.clearInterrupt();
            // Increment index only after both sensors have updated for this sample
            dataSampleIndex++;
          }
        }
      }
    }
    Serial.print("Failed get data times: ");
    Serial.println(failed_attempt);
    Serial.println("Disconnected");
  }
}
