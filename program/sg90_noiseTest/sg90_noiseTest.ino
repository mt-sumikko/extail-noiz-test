/* Sweep
  by BARRAGAN <http://barraganstudio.com>
  This example code is in the public domain.

  modified 8 Nov 2013
  by Scott Fitzgerald
  https://www.arduino.cc/en/Tutorial/LibraryExamples/Sweep
*/

#include <Servo.h>

Servo myservo;


int pos = 0;

void setup() {
  myservo.attach(9);
}

void loop() {

  for (int i = 0; i <= 10; pos += 1) {
    for (pos = 0; pos <= 180; pos += 1) {
      myservo.write(pos);
      delay(15);
    }
    for (pos = 180; pos >= 0; pos -= 1) {
      myservo.write(pos);
      delay(15);
    }
    delay(1000);


    for (pos = 0; pos <= 180; pos += 2) {
      myservo.write(pos);
      delay(15);
    }
    for (pos = 180; pos >= 0; pos -= 2) {
      myservo.write(pos);
      delay(15);
    }
    delay(1000);


    for (pos = 0; pos <= 180; pos += 5) {
      myservo.write(pos);
      delay(15);
    }
    for (pos = 180; pos >= 0; pos -= 5) {
      myservo.write(pos);
      delay(15);
    }
    delay(1000);


    for (pos = 0; pos <= 180; pos += 10) {
      myservo.write(pos);
      delay(15);
    }
    for (pos = 180; pos >= 0; pos -= 10) {
      myservo.write(pos);
      delay(15);
    }
    delay(1000);

  }
}
