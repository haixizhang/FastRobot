/**************************************************************** 
 * Complementary Filter Example for Gyroscope and Accelerometer
 * Combines gyro integration with accelerometer-based angles using
 * a complementary filter. The filter uses a lowpass idea to weight
 * the accelerometer measurement.
 *
 * Based on SparkFun ICM-20948 Example1_Basics.ino.
 *
 * Created by [Your Name], 2025
 ***************************************************************/
#include "ICM_20948.h"  // SparkFun ICM-20948 Library

//#define USE_SPI       // Uncomment to use SPI if desired

#define SERIAL_PORT Serial

#ifdef USE_SPI
  #define SPI_PORT SPI  // SPI port (if using SPI)
  #define CS_PIN 2      // Chip select pin for SPI
#else
  #define WIRE_PORT Wire  // I2C port
  // On the SparkFun 9DoF IMU breakout, AD0_VAL is 1 by default
  #define AD0_VAL 1
#endif

#ifdef USE_SPI
ICM_20948_SPI myICM;  // Using SPI
#else
ICM_20948_I2C myICM;  // Using I2C
#endif

// Global variables for gyro integration and complementary filter
static float gyro_pitch = 0.0f, gyro_roll = 0.0f, gyro_yaw = 0.0f;
static float comp_pitch = 0.0f, comp_roll = 0.0f;
static unsigned long last_time = 0;

void setup() {
  SERIAL_PORT.begin(115200);
  while (!SERIAL_PORT) {
    ;  // Wait for Serial Monitor to open
  }

#ifdef USE_SPI
  SPI_PORT.begin();
#else
  WIRE_PORT.begin();
  WIRE_PORT.setClock(400000);
#endif

  // Initialize the IMU
  bool initialized = false;
  while (!initialized) {
#ifdef USE_SPI
    myICM.begin(CS_PIN, SPI_PORT);
#else
    myICM.begin(WIRE_PORT, AD0_VAL);
#endif
    SERIAL_PORT.print(F("Initialization returned: "));
    SERIAL_PORT.println(myICM.statusString());
    if (myICM.status != ICM_20948_Stat_Ok) {
      SERIAL_PORT.println("Trying again...");
      delay(500);
    } else {
      initialized = true;
    }
  }

  // Blink LED to indicate successful initialization
  pinMode(LED_BUILTIN, OUTPUT);
  for (int i = 0; i < 3; i++) {
    digitalWrite(LED_BUILTIN, HIGH);
    delay(200);
    digitalWrite(LED_BUILTIN, LOW);
    delay(200);
  }

  // Initialize the timer for integration
  last_time = micros();
}

void loop() {
  if (myICM.dataReady()) {
    // Update sensor readings (accelerometer, gyroscope, magnetometer, temperature)
    myICM.getAGMT();
    // --- Accelerometer-Based Angle Calculation ---
    // Read accelerometer values (in mg) and convert to g.
    float ax = myICM.accX() / 1000.0f;
    float ay = myICM.accY() / 1000.0f;
    float az = myICM.accZ() / 1000.0f;
    // Compute accelerometer-based pitch and roll.
    // Based on professor's convention:
    float pitchAcc = atan2(ax, az) * 180.0f / M_PI;
    float rollAcc  = atan2(ay, az) * 180.0f / M_PI;

    // --- Lowpass Filter on Accelerometer Angles ---
    // Use static variables so that the filter "remembers" the previous state.
    static float filteredPitch = pitchAcc;  // Initialize on first call.
    static float filteredRoll = rollAcc;

    // Get current time and compute elapsed time (dt) in seconds.
    unsigned long current_time = micros();
    float dt = (current_time - last_time) / 1000000.0f;
    last_time = current_time;

    // Define the cutoff frequency in Hz.
    const float cutoff = 2.0f;  // Adjust as needed.
    // Calculate RC and the filter coefficient alpha.
    float RC = 1.0f / (2.0f * M_PI * cutoff);
    float alpha = dt / (RC + dt);

    // Update the filtered values.
    filteredPitch = filteredPitch + alpha * (pitchAcc - filteredPitch);
    filteredRoll = filteredRoll + alpha * (rollAcc - filteredRoll);

    // Integrate gyroscope rates to update angles.
    gyro_pitch += myICM.gyrX() * dt;
    gyro_roll  += myICM.gyrY() * dt;
    gyro_yaw   += myICM.gyrZ() * dt;

    // --- Complementary Filter ---
    // Here, alpha_g is the weight given to the accelerometer measurement.
    // A typical value might be around 0.02 (i.e. 98% weight to gyro integration).
    const float alpha_g = 0.02f;
    comp_pitch = gyro_pitch * (1.0f - alpha_g) + filteredPitch * alpha_g;
    comp_roll  = gyro_roll  * (1.0f - alpha_g) + filteredRoll * alpha_g;

    // --- Output Results ---
    SERIAL_PORT.print("Accel Pitch: ");
    SERIAL_PORT.print(filteredPitch, 2);
    SERIAL_PORT.print(" deg, Accel Roll: ");
    SERIAL_PORT.println(filteredRoll, 2);
    // SERIAL_PORT.print(" deg; Gyro Pitch: ");
    // SERIAL_PORT.print(gyro_pitch, 2);
    // SERIAL_PORT.print(" deg, Gyro Roll: ");
    // SERIAL_PORT.print(gyro_roll, 2);
    // SERIAL_PORT.print(" deg; Comp Pitch: ");
    // SERIAL_PORT.print(comp_pitch, 2);
    // SERIAL_PORT.print(" deg, Comp Roll: ");
    // SERIAL_PORT.print(comp_roll, 2);
    // SERIAL_PORT.println(" deg");

    delay(100);
  } else {
    SERIAL_PORT.println("Waiting for data");
    delay(500);
  }
}
