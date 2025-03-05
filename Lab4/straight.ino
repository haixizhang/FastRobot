// Define motor driver pins
const int MOTOR_BLUE_FWD   = 6;   // Blue side motor (e.g., left)
const int MOTOR_BLUE_REV   = 7;
const int MOTOR_OTHER_FWD  = 13;  // Other motor (e.g., right)
const int MOTOR_OTHER_REV  = 14;

// Test settings
const int MAX_PWM_OTHER = 65;    // Maximum PWM for the motor that requires higher PWM
// Calibrate blue motor: scale by 30/40 so that when the other motor gets 60, blue gets ~45.
const int MAX_PWM_BLUE  = (int)(MAX_PWM_OTHER * 34.0 / 40.0);  // ~30
unsigned long start_time;

void setup() {
  // Initialize motor control pins as outputs
  pinMode(MOTOR_BLUE_FWD, OUTPUT);
  pinMode(MOTOR_BLUE_REV, OUTPUT);
  pinMode(MOTOR_OTHER_FWD, OUTPUT);
  pinMode(MOTOR_OTHER_REV, OUTPUT);
  // Record the start time
  start_time = millis();
}

void stopMotors() {
  // Stop all motor outputs
  analogWrite(MOTOR_BLUE_FWD, 0);
  analogWrite(MOTOR_BLUE_REV, 0);
  analogWrite(MOTOR_OTHER_FWD, 0);
  analogWrite(MOTOR_OTHER_REV, 0);
}
void loop() {
  unsigned long current_time = millis();
  // For the first 3 seconds, keep the motors stopped.
  if ((current_time - start_time) <= 3000) {
    stopMotors();
  }
  // From 3 to 6 seconds, drive straight.
  else if ((current_time - start_time) <= 6000) {
    // Drive blue-side motor (calibrated) forward
    analogWrite(MOTOR_BLUE_FWD, MAX_PWM_BLUE);
    analogWrite(MOTOR_BLUE_REV, 0);
    // Drive the other motor forward
    analogWrite(MOTOR_OTHER_FWD, MAX_PWM_OTHER);
    analogWrite(MOTOR_OTHER_REV, 0);
  }
  // After 6 seconds, stop the motors.
  else {
    stopMotors();
  }
}
