/* Sweep
 by BARRAGAN <http://barraganstudio.com>
 This example code is in the public domain.

 modified 8 Nov 2013
 by Scott Fitzgerald
 https://www.arduino.cc/en/Tutorial/LibraryExamples/Sweep
*/

#include <Servo.h>

Servo myservo;  // create Servo object to control a servo
// twelve Servo objects can be created on most boards

int pos = 0;    // variable to store the servo position

void setup() {
  myservo.attach(9);  // attaches the servo on pin 9 to the Servo object

  myservo.write(90);  // Start at center
  delay(1000);  
}

void loop() {
  myservo.write(25);   // Try 5, 10, 15, etc.
  delay(500);
  
  // Test maximum position - decrease if it hits the other limit
  myservo.write(185); // Try 175, 170, 165, etc.
  delay(500);  
  // myservo.write(0);              // tell servo to go to position in variable 'pos'
  // delay(1500);
  // myservo.write(90);              // tell servo to go to position in variable 'pos'
  // delay(1500);
  // myservo.write(160);              // tell servo to go to position in variable 'pos'
  // delay(1500);
  // myservo.write(90);              // tell servo to go to position in variable 'pos'
  // delay(1500);

  // for (pos = 0; pos <= 180; pos += 1) { // goes from 0 degrees to 180 degrees
  //   // in steps of 1 degree
  //   myservo.write(pos);              // tell servo to go to position in variable 'pos'
  //   delay(15);                       // waits 15 ms for the servo to reach the position
  // }
  // for (pos = 180; pos >= 0; pos -= 1) { // goes from 180 degrees to 0 degrees
  //   myservo.write(pos);              // tell servo to go to position in variable 'pos'
  //   delay(15);                       // waits 15 ms for the servo to reach the position
  // }
}
