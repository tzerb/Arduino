#include <WiFiS3.h>

#include <ArduinoBLE.h>
#include <Servo.h>
#include "ArduinoGraphics.h"
#include <Arduino_LED_Matrix.h>
#include <WiFiS3.h>

Servo myservo;
ArduinoLEDMatrix matrix;

BLEService ledService("19B10000-E8F2-537E-4F6C-D104768A1214");
BLEByteCharacteristic servoChar("19B10002-E8F2-537E-4F6C-D104768A1214", BLERead | BLEWrite);

void ShowStartup() {
  unsigned long lastLedToggle = 0;
  unsigned long lastServoMove = 0;
  int servoPos = 10;
  int flashCount = 0;
  bool ledState = false;

  while (servoPos <= 170 || flashCount < 6) {
    unsigned long now = millis();

    if (flashCount < 6 && now - lastLedToggle >= 200) {
      lastLedToggle = now;
      ledState = !ledState;
      digitalWrite(LED_BUILTIN, ledState ? HIGH : LOW);

      matrix.beginDraw();
      if (ledState) {
        matrix.stroke(0xFFFFFFFF);
        matrix.fill(0xFFFFFFFF);
      } else {
        matrix.stroke(0x00000000);
        matrix.fill(0x00000000);
      }
      matrix.rect(0, 0, 12, 8);
      matrix.endDraw();
      flashCount++;
    }

    if (servoPos <= 170 && now - lastServoMove >= 5) {
      lastServoMove = now;
      myservo.write(servoPos);
      servoPos++;
    }
  }

  digitalWrite(LED_BUILTIN, LOW);
  matrix.beginDraw();
  matrix.stroke(0x00000000);
  matrix.fill(0x00000000);
  matrix.rect(0, 0, 12, 8);
  matrix.endDraw();
}

void setup() {
  Serial.begin(115200);

  pinMode(LED_BUILTIN, OUTPUT);

  matrix.begin();
  myservo.attach(9);

  ShowStartup();

  while (!BLE.begin()) {
    delay(1000);
  }
  // Reduce supervision timeout (units of 10ms, so 200 = 2 seconds)
  BLE.setConnectionInterval(6, 12);  // min/max connection interval
  BLE.setSupervisionTimeout(200);    // 2 second timeout instead of default ~4-6s

  BLE.setLocalName("R4 LED");
  BLE.setAdvertisedService(ledService);
  ledService.addCharacteristic(servoChar);
  BLE.addService(ledService);
  BLE.advertise();
}

void loop() {
  BLEDevice central = BLE.central();

  if (central) {
    digitalWrite(LED_BUILTIN, HIGH);
    while (central.connected()) {
      if (servoChar.written()) {
        int servoVal = servoChar.value(); // - '0';
        int angle = servoVal * 20;
        myservo.write(angle);
        Serial.println(servoVal);
        matrix.beginDraw();
        matrix.stroke(0xFFFFFFFF);
        matrix.textFont(Font_5x7);
        matrix.beginText(0, 1, 0xFFFFFF);
        matrix.print(servoVal);
        matrix.endText();
        matrix.endDraw();
      }
    }
    digitalWrite(LED_BUILTIN, LOW);
  }
}