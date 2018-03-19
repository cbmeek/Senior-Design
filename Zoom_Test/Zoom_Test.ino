#include <Servo.h>
Servo zoom_servo;  // create servo object to control a servo
const int offset=-8;
void setup() {
   zoom_servo.attach(10);  // attaches the servo on pin 9 to the servo object
}

void loop() {
//  zoom_servo.write(96+offset);  // zoom position
//  delay(10000);
  zoom_servo.write(90+offset);   // center position
  delay(100);
//  zoom_servo.write(81+offset);  // zoom out position
//  delay(10000);


}

