/*
  Explore Lower Limit PWM for Movement and On-Axis Turns

  This sketch gradually increases the PWM signal from a low value until the car begins to:
    1. Move forward (both motors forward)
    2. Perform an on-axis turn (one motor forward, one motor in reverse)
 
  Use the Serial Monitor (9600 baud) to observe the PWM values applied.
  Note: It may require a slightly higher PWM to start from rest than to keep the car moving.
 
  Motor wiring:
    - Motor 1: Forward on pin 6, Reverse on pin 7
    - Motor 2: Forward on pin 13, Reverse on pin 14

  All grounds must be connected (Artemis, motor drivers, and external power supply).
*/


void setup() {
  Serial.begin(9600);
 
  // Initialize motor control pins as outputs
  pinMode(6, OUTPUT);   // Motor 1 forward
  pinMode(7, OUTPUT);   // Motor 1 reverse
  pinMode(13, OUTPUT);  // Motor 2 forward
  pinMode(14, OUTPUT);  // Motor 2 reverse
}


void loop() {
  // ----- Test Forward Motion Threshold -----
  Serial.println("Testing forward motion threshold:");
  for (int pwm = 10; pwm < 70; pwm += 5) {  // Start at low PWM value
    Serial.print("Forward PWM = ");
    Serial.println(pwm);
   
    // Drive both motors forward
    analogWrite(6, pwm);
    analogWrite(7, 0);
    analogWrite(13, pwm);
    analogWrite(14, 0);
   
    delay(2000); // Run for 2 seconds to observe movement
    // Stop motors
    analogWrite(6, 0);
    analogWrite(7, 0);
    analogWrite(13, 0);
    analogWrite(14, 0);
   
    delay(3000); // Wait 3 seconds between tests
  }
 
  // Pause before switching test modes
  delay(5000);
 
  // ----- Test On-Axis Turning Threshold -----
  Serial.println("Testing on-axis turn threshold:");
  for (int pwm = 50; pwm < 150; pwm += 5) {
    Serial.print("Turn PWM = ");
    Serial.println(pwm);
   
    // For on-axis turning, run Motor 1 forward and Motor 2 in reverse
    analogWrite(6, pwm);   // Motor 1 forward
    analogWrite(7, 0);
    analogWrite(13, 0);    // Motor 2 forward off
    analogWrite(14, pwm);  // Motor 2 reverse
   
    delay(2000); // Run for 2 seconds to observe the turning motion
    // Stop motors
    analogWrite(6, 0);
    analogWrite(7, 0);
    analogWrite(13, 0);
    analogWrite(14, 0);
   
    delay(3000); // Wait 3 seconds between tests
  }
 
  // Pause before repeating the cycle
  delay(5000);
}
