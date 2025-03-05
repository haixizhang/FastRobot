/*
  Short Run Test on Ground with Auto-Stop Timer


  This sketch is designed for testing your car when it is installed in its chassis.
  It drives both motors forward at a set PWM speed for a short duration (e.g., 2 seconds),
  then stops the motors automatically to prevent uncontrolled movement.


  Motor wiring:
  - Motor 1: Forward control on pin 6, reverse on pin 7
  - Motor 2: Forward control on pin 13, reverse on pin 14


  Ensure that:
  - All grounds (Artemis, battery, motor drivers) are connected together.
  - The motor driver is powered by an external supply (or battery) as per your setup.
*/


void setup() {
  // Initialize motor control pins as outputs
  pinMode(6, OUTPUT);   // Motor 1 forward
  pinMode(7, OUTPUT);   // Motor 1 reverse
  pinMode(13, OUTPUT);  // Motor 2 forward
  pinMode(14, OUTPUT);  // Motor 2 reverse
}


void loop() {
  // Start both motors in the forward direction
  // Adjust the PWM value (here, 200) as needed for desired speed
  analogWrite(6, 50);  // Motor 1 forward speed
  analogWrite(7, 0);    // Ensure Motor 1 reverse is off
  analogWrite(13, 50); // Motor 2 forward speed
  analogWrite(14, 0);   // Ensure Motor 2 reverse is off


  // Run for a short duration (e.g., 2 seconds)
  delay(2000);


  // Stop the motors by setting all motor control pins to 0
  analogWrite(6, 0);
  analogWrite(7, 0);
  analogWrite(13, 0);
  analogWrite(14, 0);


  delay(10000);
}
