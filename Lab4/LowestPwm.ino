// Define motor driver pins 
const int MOTOR_BLUE_FWD   = 6;   // Left (blue) motor forward
const int MOTOR_BLUE_REV   = 7;
const int MOTOR_OTHER_FWD  = 13;  // Right (other) motor forward
const int MOTOR_OTHER_REV  = 14;

// LED indicator pin (blinks when PWM decreases)
const int LED_PIN = 9;  

// Start at 40 PWM (to overcome static friction)
int pwm_left  = 40;  // Initial PWM for left motor
int pwm_right = 40;  // Initial PWM for right motor

void setup() {
  // Initialize motor control pins as outputs
  pinMode(MOTOR_BLUE_FWD, OUTPUT);
  pinMode(MOTOR_BLUE_REV, OUTPUT);
  pinMode(MOTOR_OTHER_FWD, OUTPUT);
  pinMode(MOTOR_OTHER_REV, OUTPUT);

  // LED setup
  pinMode(LED_PIN, OUTPUT);

  // Start Serial Monitor
  Serial.begin(9600);
  Serial.println("Starting test: Finding the lowest sustaining PWM...");
}

void loop() {
  // Start moving at initial PWM to overcome static friction
  analogWrite(MOTOR_BLUE_FWD, pwm_left);
  analogWrite(MOTOR_BLUE_REV, 0);
  analogWrite(MOTOR_OTHER_FWD, pwm_right);
  analogWrite(MOTOR_OTHER_REV, 0);
  delay(1000); // Run at full initial PWM for 1 seconds
  Serial.println("Starting speed reduction...");
  // Gradually reduce PWM in steps of 2 until the robot stops
  while (pwm_left > 0 && pwm_right > 0) {
    pwm_left -= 2;
    pwm_right -= 2;
    // Apply the new PWM values
    analogWrite(MOTOR_BLUE_FWD, pwm_left);
    analogWrite(MOTOR_OTHER_FWD, pwm_right);
    delay(1000);  
    // Print current PWM values
    Serial.print("Current PWM: Left = ");
    Serial.print(pwm_left);
    Serial.print(", Right = ");
    Serial.println(pwm_right);
  }
  // Stop the motors
  analogWrite(MOTOR_BLUE_FWD, 0);
  analogWrite(MOTOR_OTHER_FWD, 0);
  Serial.println("Test complete.");
  while (true);  // Halt execution after test
}
