// Define motor driver pins 
const int MOTOR_BLUE_FWD   = 6;   // Left (blue) motor forward
const int MOTOR_BLUE_REV   = 7;   // Left (blue) motor reverse
const int MOTOR_OTHER_FWD  = 13;  // Right (other) motor forward
const int MOTOR_OTHER_REV  = 14;  // Right (other) motor reverse

// Test settings for forward driving
const int MAX_PWM_OTHER = 65;    // Right motor forward PWM (calibrated higher)
const int MAX_PWM_BLUE  = (int)(MAX_PWM_OTHER * 35.0 / 40.0);  // Left motor forward PWM (calibrated)

// Test settings for turning (pivot turns)
// For a left pivot turn: left motor runs in reverse and right motor runs forward.
const int TURN_PWM_LEFT  = 100;   // PWM for motor running in reverse during turn
const int TURN_PWM_RIGHT = 130;   // PWM for motor running forward during turn

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
  unsigned long elapsed = millis() - start_time;
  if (elapsed <= 3000) {
    // 0-3 sec: Stop
    stopMotors();
  } 
  else if (elapsed <= 5000) {
    // 3-5 sec: Drive forward straight
    analogWrite(MOTOR_BLUE_FWD, MAX_PWM_BLUE);
    analogWrite(MOTOR_BLUE_REV, 0);
    analogWrite(MOTOR_OTHER_FWD, MAX_PWM_OTHER);
    analogWrite(MOTOR_OTHER_REV, 0);
  } 
  else if (elapsed <= 7000) {
    // 5-7 sec: Left pivot turn:
    analogWrite(MOTOR_BLUE_FWD, 0);
    analogWrite(MOTOR_BLUE_REV, TURN_PWM_LEFT);
    analogWrite(MOTOR_OTHER_FWD, TURN_PWM_RIGHT);
    analogWrite(MOTOR_OTHER_REV, 0);
  } 
  else if (elapsed <= 9000) {
    // 7-9 sec: Right pivot turn:
    analogWrite(MOTOR_BLUE_FWD, TURN_PWM_RIGHT);
    analogWrite(MOTOR_BLUE_REV, 0);
    analogWrite(MOTOR_OTHER_FWD, 0);
    analogWrite(MOTOR_OTHER_REV, TURN_PWM_LEFT);
  } 
  else if (elapsed <= 11000) {
    // 9-11 sec: Reverse straight
    // Both motors run in reverse.
    analogWrite(MOTOR_BLUE_FWD, 0);
    analogWrite(MOTOR_BLUE_REV, MAX_PWM_BLUE);
    analogWrite(MOTOR_OTHER_FWD, 0);
    analogWrite(MOTOR_OTHER_REV, MAX_PWM_OTHER);
  } 
  else {
    // After 11 sec: Stop
    stopMotors();
  }
}
