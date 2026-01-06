#include "ArduinoGraphics.h" 
#include <Arduino_LED_Matrix.h>

// Simple smiley face pattern (12x8 matrix)
const uint32_t smiley[] = {
  0x70F0F0F0, //0x19819,
  0xF0F0F0F0, //0x80000001,
  0xF0F0F0F1 //0x81f8000
};

// Heart pattern
const uint32_t heart[] = {
  0x3184a444,
  0x44042081,
  0x100a0040
};

/*
 Stepper Motor Control - one revolution

 This program drives a unipolar or bipolar stepper motor.
 The motor is attached to digital pins 8 - 11 of the Arduino.

 The motor should revolve one revolution in one direction, then
 one revolution in the other direction.


 Created 11 Mar. 2007
 Modified 30 Nov. 2009
 by Tom Igoe

 */

#include <Stepper.h>
ArduinoLEDMatrix matrix;

const int stepsPerRevolution = 200;  // change this to fit the number of steps per revolution
// for your motor

// initialize the stepper library on pins 8 through 11:
Stepper myStepper(stepsPerRevolution, 8, 9, 10, 11);
#define LED_BUILTIN 1
#define STEP_PIN 8
#define DIR_PIN 7

const int BUTTON_PIN = A5;
volatile bool buttonPressed = false;
volatile int numPressed = 0;
volatile unsigned long lastInterruptTime = 0;

void setup() {
  // set the speed at 60 rpm:
  myStepper.setSpeed(60);

  // initialize the serial port:
  Serial.begin(115200);

  matrix.begin();  
  pinMode(STEP_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);
  pinMode(A0, OUTPUT);

  pinMode(BUTTON_PIN, INPUT_PULLUP);  // Use internal pull-up resistor
  
  // Attach interrupt - trigger on falling edge (button press)
  attachInterrupt(digitalPinToInterrupt(BUTTON_PIN), buttonISR, HIGH);
  
  Serial.println("Button interrupt ready on A5");  
}

void buttonISR() {
  unsigned long currentTime = millis();
  
  // Only register press if it's been > 200ms since last press
  if (currentTime - lastInterruptTime > 200) {
    buttonPressed = true;
    lastInterruptTime = currentTime;
    numPressed++;
  }  
}

void graphLine(int y, int x) {
  matrix.beginDraw();
  matrix.stroke(0x00000000);
  matrix.line(0, y, 11, y);
  if (x+1 > 1) {
    matrix.stroke(0xFFFFFFFF);
    matrix.line(0, y, x-1, y);
  }
  matrix.endDraw();
}

void loop() {

  int value = analogRead(A0);
  int relValue = (value * 12) / 1024;
  // Convert to string and append
  if (digitalRead(BUTTON_PIN))
  {
    graphLine(4, 12);
  }
  else
  {
    graphLine(4, 0);
  }
  graphLine(2, relValue);

  delay(50);
  // matrix.beginDraw();
  // matrix.stroke(0x00000000);  // or stroke(1)
  // matrix.line(0, 0, 11, 0);
  // matrix.stroke(0xFFFFFFFF);  // or stroke(1)
  // matrix.line(0, 2, 11, 2);
  // matrix.endDraw();
  //delay(50);

  if (buttonPressed) {
    Serial.println("Button was pressed!");
    Serial.println(numPressed);  
    buttonPressed = false;  // Reset flag
  }  
}

