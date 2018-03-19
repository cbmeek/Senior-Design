#include <Servo.h>
Servo pan_servo;
const int panOffset=0;
const int n=1;
int pin = 2;
int error,output,targetAngle,offset,count;
int angle, angleTotal, angleS[n];
unsigned long durationHigh,durationLOW,dutyCycle,angleRaw,millisLast=0,angleCheck=0;



void setup()
{
  pan_servo.attach(8);
  pinMode(pin, INPUT);
  Serial.begin(9600);
  while(!Serial){
  }
  for (int i=0; i < n; i++) angleS[i] = 0;
}

void loop()
{
   unsigned long start = millis();
   if((millis()-angleCheck)>=1){
  durationLOW = pulseIn(pin, LOW);
  durationHigh = pulseIn(pin, HIGH);
  dutyCycle = (1000*durationHigh)/(durationHigh+durationLOW);
  angleRaw = (36000*(dutyCycle - 29)/(971-30));
  if(angleRaw > 36000) {
    angleRaw = 36000;
  }
   angleRaw = angleRaw / 100;
   
   angleTotal -= angleS[count];
   angleS[count]=angleRaw;
   angleTotal += angleS[count];
   count+=1;
   if (count>=n) count = 0;
   angle = angleTotal/n;
   
  targetAngle = 10;
  error = targetAngle-angle;
  output = -200*error/360;
  Serial.print(millis()-start);
  Serial.print("  error ");
  Serial.print(error);
  Serial.print("\t");
  Serial.print("angle ");
  Serial.println(angle,DEC);
  angleCheck= millis();
} 
  
  if((millis()-millisLast)>=20){
  pan_servo.writeMicroseconds(1540);
  millisLast = millis();
  }
  

}


