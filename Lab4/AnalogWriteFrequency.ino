const int pwmPin = 6;           // PWM pin to test (50% duty cycle)
unsigned long lastRiseTime = 0; // Time of the last rising edge (in µs)
unsigned long sumPeriod = 0;    // Sum of measured periods (in µs)
int count = 0;                  // Count of measured cycles
int lastState = LOW;            // Last state read from the PWM pin

void setup() {
  Serial.begin(9600);
  // Set the pin to output and start PWM at 50% duty cycle
  pinMode(pwmPin, OUTPUT);
  analogWrite(pwmPin, 128);  // 50% duty cycle (range 0-255)
}

void loop() {
  // Read the current state of the PWM pin
  int currentState = digitalRead(pwmPin);

  // Check for a state change (edge detection)
  if (currentState != lastState) {
    // Detect rising edge: LOW -> HIGH transition
    if (currentState == HIGH) {
      unsigned long now = micros();
      if (lastRiseTime != 0) {
        // Calculate the period of one complete PWM cycle
        unsigned long period = now - lastRiseTime;
        sumPeriod += period;
        count++;
        // After 10 cycles, calculate and print the average frequency
        if (count >= 10) {
          float avgPeriod = sumPeriod / (float) count;
          float frequency = 1000000.0 / avgPeriod;
          Serial.print("Measured Frequency: ");
          Serial.print(frequency);
          Serial.println(" Hz");
          // Reset for the next measurement cycle
          sumPeriod = 0;
          count = 0;
        }
      }
      lastRiseTime = now;
    }
    lastState = currentState;
  }
}


