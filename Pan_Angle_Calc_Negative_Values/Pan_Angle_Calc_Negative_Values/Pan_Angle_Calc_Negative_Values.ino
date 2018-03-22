  #include <Servo.h>;
  Servo pan_servo;
  const int anglePer = 1000; //rounds off jitter in the signal higher number more rounding
  const int feedbackPin = 2;
  unsigned long durationLow=0, durationHigh=0, millisLast=0;
  long rotation=0, angle;
  unsigned int dutyCycle=0,theta=0;
  int newQuadrant=1, oldQuadrant=1;

void setup() {
pan_servo.attach(8);
pinMode(feedbackPin, INPUT);
Serial.begin(9600);
while(!Serial){
  }
}

void loop() {
  // Calculate Duty Cycle
  durationLow = pulseIn(feedbackPin, LOW); //Measures the time the feedback signal is low
  durationHigh = pulseIn(feedbackPin, HIGH); //Measures the time the feedback signal is high
  dutyCycle = (10000*durationHigh)/(durationHigh+durationLow); //calculates the duty cycle
 
  //Formula for angle calculation taken from parallax 360 feedback data sheet
  theta = (35900*(dutyCycle - 290))/(9710-290+1)/100;  
  if(theta > 359) theta = 359; //limits theta to bounds

  //Determines what quadrant the servo rotates from to determine positive and negative angles outside 0-359
  if( theta>=0 && theta<=90){
    oldQuadrant = newQuadrant; 
    newQuadrant = 1;
  }
  if( theta>90 && theta<=180){
    oldQuadrant = newQuadrant; 
    newQuadrant = 2;
  }
    if( theta>180 && theta<=270){
    oldQuadrant = newQuadrant; 
    newQuadrant = 3;
  }
  else if( theta>270 && theta<=359 ){
   oldQuadrant = newQuadrant; 
   newQuadrant = 4;
  }
  if(oldQuadrant==1 && newQuadrant==4){ 
    rotation = rotation - 1; 
    }
  else if( oldQuadrant==4 && newQuadrant==1){
    rotation = rotation + 1;
    }
    
  angle = 360*rotation + theta;
 
 //Print the Data 
  Serial.print("newQuadrant: ");
  Serial.print(newQuadrant);
  Serial.print("\t");
  Serial.print("oldQuadrant: ");
  Serial.print(oldQuadrant);
  Serial.print("\t");
  Serial.print("Theta: ");  
  Serial.print(theta,DEC);
  Serial.print("\t");
  Serial.print("Rotation #: ");
  Serial.print(rotation);
  Serial.print("\t");
  Serial.print("Angle: ");
  Serial.println(angle,DEC);

  
    if((millis()-millisLast)>=20){
      pan_servo.writeMicroseconds(1538);
      millisLast = millis();  
    }
        
}
