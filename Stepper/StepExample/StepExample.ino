#include <AccelStepper.h>

// Define pins (adjust to your wiring)
#define STEP_PIN 8
#define DIR_PIN 7
#define ENABLE_PIN 4    // ENA+ on TB6600 (optional, for motor enable/disable)

// ===== MOTOR SPECIFICATIONS =====
// 17HS4401: 200 steps/revolution (1.8° per step)
const int STEPS_PER_REV = 200;

// ===== TB6600 MICROSTEP SETTING =====
// Set TB6600 DIP switches to 1/8 microstep (good balance of speed and smoothness)
// This means 200 * 8 = 1600 steps per revolution
const int MICROSTEPS = 8;
const int STEPS_PER_REV_MICROSTEPPED = STEPS_PER_REV * MICROSTEPS;  // 1600

// Create stepper object
AccelStepper stepper(AccelStepper::DRIVER, STEP_PIN, DIR_PIN);

void setup() {
  Serial.begin(115200);
  
  // Set up enable pin
  pinMode(ENABLE_PIN, OUTPUT);
  digitalWrite(ENABLE_PIN, LOW);  // LOW = motor enabled, HIGH = motor disabled
  
  // ===== SPEED AND ACCELERATION SETTINGS =====
  // At 12V with 1/8 microstep, these are good starting values:
  
  stepper.setMaxSpeed(2000);        // Maximum steps per second (adjust as needed)
                                     // At 1600 steps/rev, this is 1.25 rev/sec = 75 RPM
  
  stepper.setAcceleration(1000);    // Steps per second per second
                                     // Smooth acceleration prevents stalling
  
  stepper.setCurrentPosition(0);    // Set current position as zero
  
  Serial.println("Stepper motor ready!");
  Serial.println("Max speed: 2000 steps/sec = 75 RPM");
}

void loop() {
  // Example 1: Move forward 5 revolutions
  Serial.println("Moving forward 5 revolutions...");
  stepper.moveTo(5 * STEPS_PER_REV_MICROSTEPPED);  // 5 * 1600 = 8000 steps
  while(stepper.distanceToGo() != 0) {
    stepper.run();
  }
  
  delay(1000);
  
  // Example 2: Move backward 5 revolutions  
  Serial.println("Moving backward 5 revolutions...");
  stepper.moveTo(0);  // Return to zero position
  while(stepper.distanceToGo() != 0) {
    stepper.run();
  }
  
  delay(1000);
  
  // Example 3: Continuous rotation at constant speed
  Serial.println("Continuous rotation at 60 RPM...");
  stepper.setSpeed(1600);  // 1600 steps/sec = 1 rev/sec = 60 RPM
  
  for(int i = 0; i < 10000; i++) {  // Run for a while
    stepper.runSpeed();
  }
  
  delay(1000);
  
  // Example 4: Disable motor (saves power, but motor can be turned by hand)
  // digitalWrite(ENABLE_PIN, HIGH);
  // delay(5000);
  // digitalWrite(ENABLE_PIN, LOW);
}
/*
```
## TB6600 DIP Switch Settings

Set the switches on your TB6600 like this:

**Microstep Setting (SW5, SW6, SW7):**
- For **1/8 microstep**: OFF, ON, OFF
  - This gives you 1600 steps/revolution
  - Good balance of smoothness and speed at 12V

**Current Setting (SW1, SW2, SW3):**
- For **1.5A** (safe for 17HS4401): ON, OFF, ON
- The 17HS4401 is rated around 1.7A, so 1.5A is a good setting
```
Common TB6600 Microstep Settings:
SW5  SW6  SW7  | Microstep
ON   ON   ON   | 1/2
OFF  ON   ON   | 1/4
ON   OFF  ON   | 1/8  ← RECOMMENDED
OFF  OFF  ON   | 1/16
AccelStepper stepper(AccelStepper::DRIVER, STEP_PIN, DIR_PIN);

void setup() {
  stepper.setMaxSpeed(2000);      // Steps per second
  stepper.setAcceleration(1000);  // Steps per second per second
  pinMode(DIR_PIN, OUTPUT);
}

bool dir = false;
void loop() {
  stepper.moveTo(10000);  // Move 10000 steps
  stepper.run();          // Must call repeatedly
  digitalWrite(DIR_PIN, dir? HIGH : LOW);  // Clockwise
  dir = !dir;
}
*/