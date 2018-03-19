#include <Servo.h>
Servo pan_servo;
Servo tilt_servo;
Servo zoom_servo;
const int panOffset=0;
const int tiltOffset=0;
const int zoomOffset=-10;
const int n=1;
int increment=1;
int pin = 2;
int zoom=4,error,output,targetAngle,offset,count,tiltAngle=90;
int angle, angleTotal, angleS[n];
unsigned long zoomUpdate,tiltUpdate,durationHigh,durationLOW,dutyCycle,angleRaw,millisLast=0,angleCheck=0;



void setup()
{
  pan_servo.attach(8);
  tilt_servo.attach(9);
  zoom_servo.attach(10);
  pinMode(pin, INPUT);
  Serial.begin(115200);
  Serial1.begin(9600);

  while(!Serial){
  }
  for (int i=0; i < n; i++) angleS[i] = 0;
}

void loop()
{
    //Change tilt angle
  if((millis() - tiltUpdate)>=200){
    tiltAngle += increment;
    tilt_servo.write(tiltAngle+tiltOffset);
    if((tiltAngle >= 100) ||(tiltAngle <= 60)){
      increment=-increment;
    }
      tiltUpdate = millis();  
  }
  
  if((millis()-millisLast)>=20){
    if(angle >= 300){
    output = 1532; 
    }
    else if(angle <= 150){
     output = 1448;
     }
   pan_servo.writeMicroseconds(output);
   millisLast = millis();
  }

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
  /*
  Serial.print("angle ");
  Serial.println(angle,DEC);
  targetAngle = 10;
  error = targetAngle-angle;
  output = -200*error/360;
  Serial.print(millis()-start);
  Serial.print("  error ");
  Serial.print(error);
  Serial.print("\t");
  Serial.print("angle ");
  Serial.println(angle,DEC);
  */
  angleCheck= millis();
}
 if(millis() - zoomUpdate >= 5000){
   
   zoom_servo.write(90+zoom+zoomOffset);
   zoomUpdate =millis();
   zoom=-zoom;
 }
  //Display GPS Data stream
  if(Serial1.available()){
    Serial.write(Serial1.read());
  }
}


