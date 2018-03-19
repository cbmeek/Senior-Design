#include <Servo.h>
Servo pan_servo;  // create servo object to control a servo
Servo tilt_servo;
Servo zoom_servo;

const int tiltOffset=0;
const int zoomOffset=-8;
const int panOffset=0;
int angle;

void setup() {
  pan_servo.attach(8);
  tilt_servo.attach(9);
  zoom_servo.attach(10);
  
}

void loop() {
    if((milies() - tiltUpdate) > 40){
     tiltUpdate = millis();
     angle += increment;
     tilt_servo.write(angle+tiltOffset);
     if((angle >= 180) ||(pos <= 45)){
      increment=-increment;
     }
    }
   if((milies() - panUpdate) > 1500){
    panUpdate = milis();
    pan_servo.writeMicroseconds(1550+panOffset);
    zoom_servo.write(97+zoomOffset);
   }
   if((milies() - pan2Update) > 3000){
    panUpdate = milis();
    pan_servo.writeMicroseconds(1420+panOffset);
    zoom_servo.write(80+zoomOffset);
   }   
   if((milies()
   delay(1500);
   pan_servo.writeMicroseconds(1550+panOffset);
   zoom_servo.write(97+zoomOffset);
   delay(1500);
   pan_servo.writeMicroseconds(1500+panOffset);
   zoom_servo.write(97+zoomOffset);
   delay(1500);
   pan_servo.writeMicroseconds(1420+panOffset);
   zoom_servo.write(80+zoomOffset);
   delay(1500);
   pan_servo.writeMicroseconds(1420+panOffset);
   zoom_servo.write(80+zoomOffset);
   delay(1500);
   pan_servo.writeMicroseconds(1500+panOffset);
   zoom_servo.write(90+zoomOffset);
   delay(1500);
 
}


