#include <AccelStepper.h>

// ULN2003 Driver Board pins (IN1, IN2, IN3, IN4)
#define IN1 8
#define IN2 9
#define IN3 10
#define IN4 11

// 28BYJ-48 stepper motor
// Use HALF4WIRE for smoother operation
AccelStepper stepper(AccelStepper::HALF4WIRE, IN1, IN3, IN2, IN4);

// 28BYJ-48 specifications:
// Gear ratio: 64:1 (approximately)
// Steps per revolution (half-step mode): 4096 steps = 360°
const int STEPS_PER_REVOLUTION = 2048;  // In half-step mode

void setup() {
  Serial.begin(115200);
  
  // These motors are slow - don't go too fast!
  stepper.setMaxSpeed(1000);      // Max around 1000 steps/sec
  stepper.setAcceleration(500);   // Smooth acceleration
  
  Serial.println("28BYJ-48 Stepper Ready!");
}

void loop() {
  // Example 1: One full revolution clockwise
  Serial.println("Rotating 360° clockwise...");
  stepper.moveTo(STEPS_PER_REVOLUTION);
  while(stepper.distanceToGo() != 0) {
    stepper.run();
  }
  
  delay(1000);
  
  // Example 2: One full revolution counter-clockwise (back to start)
  Serial.println("Rotating 360° counter-clockwise...");
  stepper.moveTo(0);
  while(stepper.distanceToGo() != 0) {
    stepper.run();
  }
  
  delay(1000);
  
  // Example 3: 90° rotation
  Serial.println("Rotating 90°...");
  stepper.move(STEPS_PER_REVOLUTION / 4);  // Quarter turn
  while(stepper.distanceToGo() != 0) {
    stepper.run();
  }
  
  delay(1000);
}