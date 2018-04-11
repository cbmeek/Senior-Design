#include <Servo.h>
Servo pan_servo;  // create servo object to control a servo
Servo tilt_servo;
Servo zoom_servo;

const int tiltOffset=0;
const int zoomOffset=-8;
const int panOffset=0;

void setup() {
  pan_servo.attach(8);
  tilt_servo.attach(9);
  zoom_servo.attach(10);
   
}

void loop() {
  pan_servo.writeMicroseconds(1500+panOffset);
  tilt_servo.write(90+tiltOffset);  
  zoom_servo.write(90+zoomOffset);

}

