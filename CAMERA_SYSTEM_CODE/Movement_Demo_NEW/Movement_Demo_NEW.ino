#include <Servo.h>
Servo pan_servo;  // create servo object to control a servo
Servo tilt_servo;
Servo zoom_servo;

const int tiltOffset=0;
const int zoomOffset=-8;
const int panOffset=0;
int angle=90;
int increment = 1;
int count=0;
unsigned long tiltUpdate=0, panUpdate=0, panUpdate2=0;

void setup() {
  pan_servo.attach(8);
  tilt_servo.attach(9);
  zoom_servo.attach(10);
  Serial.begin(115200);
  Serial.println("Hello,World");
  Serial1.begin(9600);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }
  }
void loop() {
  //Counter to reverse pan direction
  if((millis() - panUpdate2) > 1) {
    if(count <= 6000) {
      count = count+1;
    }
    else {
      count = 0;
    }
    panUpdate2 = millis();
  }
  

  //Change tilt angle
  if((millis() - tiltUpdate) > 20){
    
    angle += increment;
    tilt_servo.write(angle+tiltOffset);
    if((angle >= 180) ||(angle <= 45)){
      increment=-increment;
    }
    tiltUpdate = millis();
  }
  
  //Pan camera 
  if((millis() - panUpdate) > 500){
    if (count<= 3000){
      pan_servo.writeMicroseconds(1570+panOffset);
      zoom_servo.write(83+zoomOffset);
    }
    else{
      pan_servo.writeMicroseconds(1400+panOffset);
      zoom_servo.write(96+zoomOffset);      
    }
    panUpdate = millis();
  }
  
  //Display GPS Data stream
  if(Serial1.available()){
    Serial.write(Serial1.read());
  }

}



