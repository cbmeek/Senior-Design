  #include <Servo.h>;
  Servo pan_servo;
  const int anglePer = 1000; //rounds off jitter in the signal higher number more rounding
  const int feedbackPin = 2;
  unsigned long durationLow, durationHigh, millisLast;
  long rotation=0, angle;
  unsigned int dutyCycle,theta;
  int newQuadrant,oldQuadrant;

void setup() {
pan_servo.attach(9);
pinMode(feedbackPin, INPUT);
Serial.begin(9600);
while(!Serial){
  }
}

void loop() {
  // Calculate Duty Cycle
  durationLow = pulseIn(feedbackPin, LOW); //Measures the time the feedback signal is low
  durationHigh = pulseIn(feedbackPin, HIGH); //Measures the time the feedback signal is high
  dutyCycle = (anglePer*durationHigh)/(durationHigh+durationLow); //calculates the duty cycle
 
  //Formula for angle calculation taken from parallax 360 feedback data sheet
  theta = (359*anglePer*(dutyCycle- 2.9*anglePer))/(97.1*anglePer-2.9*anglePer+1);  
  if(theta > 359*anglePer) theta = 359*anglePer; //limits theta to bounds

  //Determines if the servo rotates from 1st quadrant to the 4th or vica versa to determine angles outside 0-359
  if((theta>=0)&&(theta<=90)){ 
    newQuadrant = 1;
  }
  else if( (theta>=270)&&(theta<=359) ){
   newQuadrant = 4;
  }
  if((oldQuadrant==1) && (newQuadrant==4)){ 
    rotation = rotation - 1; 
    oldQuadrant = newQuadrant;
    }
  else if((oldQuadrant==4) && (newQuadrant==1)){
    rotation = rotation + 1;
    oldQuadrant = newQuadrant;
    }
  angle = 360*rotation + theta/anglePer; //rounds off errors in the decmial 
  Serial.println(angle,DEC);
    if((millis()-millisLast)>=20){
      pan_servo.writeMicroseconds(1540);
      millisLast = millis();
    }
        
}
