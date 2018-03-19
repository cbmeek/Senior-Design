#include <Servo.h>

int pin = 2;
unsigned long high,low,duration;
volatile unsigned long timeOne=0, timeTwo=0, timeThree=0;
volatile int state=1; 
unsigned int duty=100;
unsigned int angle=0;
Servo pan_servo;

void setup() {
  // put your setup code here, to run once:
 pinMode(pin, INPUT_PULLUP);
 attachInterrupt(0, count, CHANGE); 
 pan_servo.attach(8);
 Serial.begin(9600);
 while(!Serial){
 }
}

void count(){
 switch (state) {
  case 1:
    timeOne = timeThree;
    state = 2;
    break;
    
  case 2:
    timeTwo = micros();
    state = 3;
    break;
  
  case 3:
    timeThree = micros();
    state = 1;
    break;

    default:
    timeOne=micros();
    state = 2;
    break;
  }
}
  
void loop() {
  
  pan_servo.writeMicroseconds(1550); 
 
  if(state==3){
    duty = 100*(timeTwo-timeOne)/(timeThree-timeOne);
    angle=360*(duty - 3)/94;
  }
  
  Serial.print("Duty Cycle: ");
  Serial.println(duty,DEC);
  Serial.print("Angle: ");
  Serial.println(angle,DEC);
}
