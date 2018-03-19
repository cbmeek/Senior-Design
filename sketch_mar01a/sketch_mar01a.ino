#include <Servo.h>            //Servo library

//PID angle control for Parallax 360 Servo
Servo servo_test;        //initialize a servo object for the connected servo  
 
void setup() 
{ 
  servo_test.attach(9);      // attach the signal pin of servo to pin9 of arduino
} 
  
void loop() 
{ 
      servo_test.writeMicroseconds(1570); //Move the servo
      delay(5);
      servo_test.writeMicroseconds(1407);
      delay(5);
  
}
