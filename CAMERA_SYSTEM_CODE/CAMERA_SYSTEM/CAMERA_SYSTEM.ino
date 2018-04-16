/* Camera System Control 
 *  
 *Credit to: 
 *Robin2 - "Serial Input Basics updated" available: http://forum.arduino.cc/index.php?topic=396450
 */
#include <Servo.h>
#include <PID_v1.h>

//Constants
  const float degToRad = 3.14159265359/180;
  const float radToDeg = 1/degToRad;
  const int calPin1 = 4;
  const int calPin2 = 5;
  const int calPin3 = 6;
  const int calPin4 = 7;

  const int panFeedBackPin = 2;
  const int panPin = 8;

  const int tiltPin = 9;
  const int tiltOffset = 70;
  const int zoomPin = 10;

//holds calibration/initialization data
  float lat0=0.0;
  float lon0=0.0;
  float ele0=0.0;
  double lonCorrection=0.0;
  
//holds GPS transmistter data
  float dLat=1.0,dLatOld=1.0;
  float dLon=1.0, dLonOld=1.0;
  float dEle=1.0, dEleOld=1.0;
  float distance=1.0,distanceOld=1.0;
  
//holds Pan angle data
  const int panWeight = 80;
  int newQuadrant = 1, oldQuadrant = 1;
  long panAngle=0, panAngleOld=0;
  double doublePanAngle=0;
  unsigned long durationLow = 0, durationHigh = 0;
  long rotation = 0;
  unsigned int dutyCycle = 0, theta = 0;
  
//Pan Control data
  const double outputMax = 1620;
  const double outputMin = 1360;
  const int timeStep=80;   
  double targetPanAngle=105;
  double outputPanServo=1500;
  double Kp=16;
  double Ki=1.5;
  double Kd=.8;

//Time
  unsigned long millisStart,panMillis,panServoMillis,targetAngleINCMillis;  

//Serial parsing
  const byte stringSize = 64;
  char receivedChars[stringSize];
  char tempChars[stringSize];
  float gpsLat=0.0; 
  float gpsLon=0.0;
  float gpsEle=0.0;
  float gpsVoltage=0.0;
  boolean newData = false;
  

//Instatiate Objects  
Servo panServo;
Servo tiltServo;
Servo zoomServo;

PID panPID(&doublePanAngle, &outputPanServo, &targetPanAngle, Kp, Ki, Kd, REVERSE);

boolean trigger = true; //used to trigger the target angle loop at least once 
 
void setup() {
  //Pan Servo Setup
  panServo.attach(panPin);
  pinMode(panFeedBackPin, INPUT);
  
  //Tilt Servo Setup
  tiltServo.attach(tiltPin);
  tiltServo.writeMicroseconds(1500+tiltOffset);
  
  //Zoom Servo Setup
  zoomServo.attach(zoomPin);
 
     
  //Comunication Setup
  Serial.begin(9600);   //USB serial port
  Serial1.begin(19200); //HC12 wireless transceiver
  while(!Serial){       //Wait for USB serial port to connect 
    }
  
  //PID Setup
  panPID.SetMode(AUTOMATIC);
  panPID.SetOutputLimits(outputMin, outputMax);
  panPID.SetSampleTime(timeStep);
   
  //Timer
  millisStart=millis();
  panMillis=millis();
  targetAngleINCMillis=millis();
  panServoMillis=millis();
  
  //HMI Pins and LEDS
  pinMode(calPin1,INPUT);
  pinMode(calPin2,INPUT);
}

void loop() {
  if(digitalRead(calPin1) == HIGH) calibration();
  else if(digitalRead(calPin4)==HIGH) maintenanceMode(); 
  else if(digitalRead(calPin3)==HIGH){
    //Receieve GPS Transmitter Data
    recvSerialData(Serial1); 
    if(newData == true) {
      parseGPSData();          
    }
      
    //Determine Pan Angle
    getPanAngle();

    //Pan Angle PID
    //panPID.Compute();
    
    //Update Target Angles
    while((panAngle <= targetPanAngle+1) && (panAngle >= targetPanAngle-1) || trigger==true) {
      updateTranLoc();
      updateTargetPanAngle();
      trigger = false;
      
      if((panAngle <= targetPanAngle+1) && (panAngle >= targetPanAngle-1 )){
       outputPanServo=1500;
       dLatOld = dLat;
       dLonOld = dLon;
       dEleOld = dEle;
       distanceOld = distance;
      }
    } while((panAngle <= targetPanAngle+1) && (panAngle >= targetPanAngle-1));
        


    //Servo Control
    //panServo.writeMicroseconds((int)outputPanServo);
  
  
    //Display
    Serial.print(millis()-millisStart);  Serial.print(" ");
    Serial.print("targetPanAngle: "); Serial.print(targetPanAngle);  Serial.print(" ");
    Serial.print("panAngle: "); Serial.print(panAngle);  Serial.print(" ");
    Serial.print("outputPanServo: "); Serial.println(outputPanServo,DEC);
  
   //PID TESTING
  /* if((millis()-targetAngleINCMillis) >= 1000){
     targetPanAngle += 10;
     targetAngleINCMillis = millis();
     }
  */
  
  }//else if
  else{
    panServo.writeMicroseconds(1500);
  }
}//loop

void recvSerialData(Stream &ser) {
    static boolean recvInProgress = false;
    static byte ndx = 0;
    char startMarker = '<';
    char endMarker = '>';
    char rc;

    while (ser.available() > 0 && newData == false) {
        rc = ser.read();

        if (recvInProgress == true) {
            if (rc != endMarker) {
                receivedChars[ndx] = rc;
                ndx++;
                if (ndx >= stringSize) {
                    ndx = stringSize - 1;
                }
            }
            else {
                receivedChars[ndx] = '\0'; // terminate the string
                recvInProgress = false;
                ndx = 0;
                newData = true;
            }
        }

        else if (rc == startMarker) {
            recvInProgress = true;
        }
    }
}

void getPanAngle(){   
  durationLow = pulseIn(panFeedBackPin, LOW); //Measures the time the feedback signal is low
  durationHigh = pulseIn(panFeedBackPin, HIGH); //Measures the time the feedback signal is high
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
  panAngleOld = panAngle;   
  panAngle = (panWeight*panAngleOld + (100-panWeight)*(360*rotation + theta))/100;
  doublePanAngle = (double)panAngle; //cast the pan angle to double for pid
 /*
 //Print the Data 
  Serial1.print("newQuadrant: ");  Serial1.print(newQuadrant);  Serial1.print("\t");
  Serial1.print("oldQuadrant: ");  Serial1.print(oldQuadrant);  Serial1.print("\t");
  Serial1.print("Theta: ");  Serial1.print(theta,DEC);  Serial1.print("\t");
  Serial1.print("Rotation #: ");  Serial1.print(rotation);  Serial1.print("\t");
  Serial1.print("panAngle: ");  Serial1.println(panAngle,DEC);
 */ 
}

//Calibrate camera system
void calibration(){   
  tiltServo.writeMicroseconds(1500+tiltOffset);
  recvSerialData(Serial1); 
  if(newData == true) {
     parseGPSData();    
     trigger = true; //trigger the angle update loop on return to normal operation
      
   //SET THE LONGITUDE,LATITUDE, AND ELEVATION OF THE CAMERA SYSTEM
   if(digitalRead(calPin1)== HIGH && digitalRead(calPin2) == HIGH){   
    lat0 = gpsLat;
    ele0 = gpsEle;
    lon0 = gpsLon;
    lonCorrection = cos(degToRad*lat0); //correction for longitude values using loca, flat earth approx.
   
    Serial.print(millis()-millisStart); Serial.print("\t");
    Serial.print("lat0: ");  Serial.print(lat0,9);  Serial.print("\t");
    Serial.print("lon0: ");  Serial.print(lon0,9);  Serial.print("\t");
    Serial.print("ele0: ");  Serial.println(ele0,3);
    
   }
   //SET TRIPOD TO ALIGN WITH GPS TRANSMITTER AND CALCULATE INITIAL POSITION VECTOR
   //When aligning camera lens ensure GPS transmistter is centered about Y axis
   else if(digitalRead(calPin1)== HIGH && digitalRead(calPin2) == LOW){
    updateTranLoc();    
    Serial.print(millis()-millisStart); Serial.print("\t");
    Serial.print("dlat: ");  Serial.print(dLat,9);  Serial.print("\t");
    Serial.print("dlon: ");  Serial.print(dLon,9);  Serial.print("\t");
    Serial.print("distance: ");  Serial.print(distance,6);  Serial.print("\t");
    Serial.print("dele: ");  Serial.println(dEle,3);
      
   }
  }
}

void parseGPSData() {      // split the data into its parts   
    char * strtokIndx; // this is used by strtok() as an index
    strcpy(tempChars, receivedChars);
    strtokIndx = strtok(tempChars,",");      // get the first part - the string
    gpsLat = atof(strtokIndx); // convert this part to a float
  
    strtokIndx = strtok(NULL, ","); // this continues where the previous call left off
    gpsLon = atof(strtokIndx);     // convert this part to a float

    strtokIndx = strtok(NULL, ",");
    gpsEle = atof(strtokIndx);     // convert this part to a float
    
    strtokIndx = strtok(NULL, ",");
    gpsVoltage = atof(strtokIndx);     // convert this part to a float
    
    newData = false;
}

void updateTranLoc(){
  dLat = gpsLat-lat0;
  dLon = gpsLon- lon0;
  dLon = lonCorrection*dLon;
  dEle= gpsEle-ele0;
  distance = sqrt(dLat*dLat +dLon*dLon)*111009; //gives distance in meters
}

void updateTargetPanAngle(){
  double  vect=(dLat*dLatOld+dLon*dLonOld)/(distanceOld*distance);
  targetPanAngle = panAngle + radToDeg*arcCos(vect);  
}

void maintenanceMode(){
  panServo.writeMicroseconds(1500);
  tiltServo.write(175);  
}

//Series Solution for the trig solutions below by Abhilash Patel. Available: http://www.instructables.com/id/Arduino-Trigonometric-Inverse-Functions/
float arcSin(float c){
  float out;
  out= ((c+(c*c*c)/6+(3*c*c*c*c*c)/40+(5*c*c*c*c*c*c*c)/112+
  (35*c*c*c*c*c*c*c*c*c)/1152 +(c*c*c*c*c*c*c*c*c*c*c*0.022)+
  (c*c*c*c*c*c*c*c*c*c*c*c*c*.0173)+(c*c*c*c*c*c*c*c*c*c*c*c*c*c*c*.0139)+
  (c*c*c*c*c*c*c*c*c*c*c*c*c*c*c*c*c*0.0115)+(c*c*c*c*c*c*c*c*c*c*c*c*c*c*c*c*c*c*c*0.01)
   ));
                                           //asin
  if(c>=.96 && c<.97){out=1.287+(3.82*(c-.96)); }
  if(c>=.97 && c<.98){out=(1.325+4.5*(c-.97));}          // arcsin
  if(c>=.98 && c<.99){out=(1.37+6*(c-.98));}
  if(c>=.99 && c<=1){out=(1.43+14*(c-.99));}  
  return out;// in radians
}

float arcCos(float c){
  float out;
  out=arcSin(sqrt(1-c*c));
  return out; // in radians
}

float arcTan(float c){
  float out;
  out=arcSin(c/(sqrt(1+c*c)));
  return out; // in radians
}


