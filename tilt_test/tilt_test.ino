#include <Servo.h>
Servo tilt_servo;  // create servo object to control a servo
const int offset=0;
void setup() {
   tilt_servo.attach(9);  // attaches the servo on pin 9 to the servo object
}

void loop() {
  // tilt_servo.write(90+offset);   // level position
  // tilt_servo.write(180+offset);  // vertical position
  // tilt_servo.write(45+offset);  // down position

}

