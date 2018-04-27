/* Camera System Control 
 *  
 *Parsing Code Credit to: 
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
  long lat0=0.0;
  long lon0=0.0;
  long ele0=0.0;
  float lonCorrection=0.0;
  
//holds GPS transmistter data
  long dLat=0,dLatOld=0;
  float dLon=0, dLonOld=0;
  long dEle=0, dEleOld=0;
  long distance=0,distanceOld=0;
  
//holds Pan angle data
  const int panWeight = 80;
  int newQuadrant = 1, oldQuadrant = 1;
  long panAngle=0, panAngleOld=0;
  double doublePanAngle=0;
  unsigned long durationLow = 0, durationHigh = 0;
  long rotation = 0;
  unsigned int dutyCycle = 0, theta = 0;
  
//Pan Control data
  const double outputMax = 1600;
  const double outputMin = 1390;
  const int timeStep=100;
  const int adaptSw =15;   
  double targetPanAngle=105;
  double outputPanServo=1500;
  double KpFar=2; //2
  double KiFar=0.7;
  double KdFar=.8; //.8
  double KpClose=0;
  double KiClose=0;
  double KdClose=0; 

//Tilt Angle
int tiltAngle;

//Time
  unsigned long millisStart,panMillis,panServoMillis,targetAngleINCMillis,updatePanMillis;  

//Serial parsing
  const byte stringSize = 64;
  char receivedChars[stringSize];
  char tempChars[stringSize];
  long gpsLat=0; 
  long gpsLon=0;
  float gpsEle=0;
  float gpsEleOld=0;
  float gpsVoltage=0;
  boolean newData = false;

  long yPosition = 240;
  long xPosition = 360;
  
//Instatiate Objects  
Servo panServo;
Servo tiltServo;
Servo zoomServo;

//Signal Processing Variables
const int WinSize = 10;
int winIdx = 0;
long winLat[WinSize];
long winLon[WinSize];

//Instantiate PID Object
PID panPID(&doublePanAngle, &outputPanServo, &targetPanAngle, KpFar, KiFar, KdFar,P_ON_E, REVERSE); 

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

void parseGPSData() {      // split the data into its parts and feed signal processing buffer  
    char * strtokIndx;  // this is used by strtok() as an index
    strcpy(tempChars, receivedChars); //make a copy of the received string
    
    strtokIndx = strtok(tempChars,","); // get the first part - the string
    gpsLat = atol(strtokIndx); // convert this part to a float
  
    strtokIndx = strtok(NULL, ","); // this continues where the previous call left off
    gpsLon = atol(strtokIndx);     // convert this part to a float
 
    strtokIndx = strtok(NULL, ",");
    gpsEle = atof(strtokIndx);     // convert this part to a float

    strtokIndx = strtok(NULL, ",");
    gpsVoltage = atof(strtokIndx);     // convert this part to a float
}

void parseCVData() { //split the data up from the Rasperry pi3
    char * strtokIndx;  // this is used by strtok() as an index
    strcpy(tempChars, receivedChars); //make a copy of the received string
    
    strtokIndx = strtok(tempChars,","); // get the first part - the string
    xPosition = atol(strtokIndx); // convert this part to a float
  
    strtokIndx = strtok(NULL, ","); // this continues where the previous call left off
    yPosition = atol(strtokIndx);     // convert this part to a float
}

void updateTargetPanAngleCV(){
  //Camera Specific Calculation MUST change for other cameras
  targetPanAngle = doublePanAngle + int( atan2(1.091*(xPosition/720 - 0.5),1.0) * radToDeg ); 
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

void updateTranLoc(){
  dLat = gpsLat - lat0;
  dLon = gpsLon - lon0;
  dLon = lonCorrection*dLon;
  dEle= gpsEle-ele0;
  distance = round(sqrt((dLat)*(dLat)+ (dLon)*(dLon))*11); //gives distance in millimeters
}

//Calibrate camera system
void calibration(){   
  tiltServo.writeMicroseconds(1500+tiltOffset);
    
   //SET THE LONGITUDE,LATITUDE, AND ELEVATION OF THE CAMERA SYSTEM
   if(digitalRead(calPin1)== HIGH && digitalRead(calPin2) == HIGH){   
    lat0 = gpsLat;
    lon0 = gpsLon;
    ele0 = gpsEle;
    lonCorrection = cos(degToRad*lat0/10000000); //correction for longitude values using loca, flat earth approx.
   
    Serial.print(millis()-millisStart); Serial.print("\t");
    Serial.print("lat0: ");  Serial.print(lat0);  Serial.print("\t");
    Serial.print("lon0: ");  Serial.print(lon0);  Serial.print("\t");
    Serial.print("ele0: ");  Serial.println(ele0);
   }
   //SET TRIPOD TO ALIGN WITH GPS TRANSMITTER AND CALCULATE INITIAL POSITION VECTOR
   //When aligning camera lens ensure GPS transmistter is centered about Y axis
   else if(digitalRead(calPin1)== HIGH && digitalRead(calPin2) == LOW){
    updateTranLoc();
    getPanAngle();
    targetPanAngle = doublePanAngle;
    Serial.print(millis()-millisStart); Serial.print("\t");
    Serial.print("dlat: ");  Serial.print(dLat);  Serial.print("\t");
    Serial.print("dlon: ");  Serial.print(dLon);  Serial.print("\t");
    Serial.print("distance: ");  Serial.print(distance);  Serial.print("\t");
    Serial.print("dele: ");  Serial.println(dEle);  
   } 
}

void winLatAndLon(){
  winLat[winIdx]=gpsLat;
  winLon[winIdx]=gpsLon;
  winIdx++;
  if(winIdx >= WinSize)  winIdx = 0;
}
  
void updateTargetPanAngle(){ //updates target pan angle with a regression line connecting to the camera system through 10 gps readings 
  long regL=0;
  long top=0;
  long bot=0;
  long tempAngle;  
  for(int i=0; i<WinSize; i++){
    top += (winLat[i]-lat0)*(winLon[i]-lon0);
    bot += (winLon[i]-lat0)*(winLon[i]-lat0);
  }
  regL = top/bot;
  tempAngle = int(atan2 (top, bot) * radToDeg);
  if(regL>0 && dLon<0) targetPanAngle = (tempAngle-180)-(doublePanAngle-int(doublePanAngle/360)*360)+doublePanAngle;   
  else if(regL<0 && dLon<0)  targetPanAngle = (tempAngle+180)-(doublePanAngle-int(doublePanAngle/360)*360)+doublePanAngle;
  else if(regL==0 && dLon<0) targetPanAngle = (tempAngle+180)-(doublePanAngle-int(doublePanAngle/360)*360)+doublePanAngle;
  else targetPanAngle = (tempAngle)-(doublePanAngle-int(doublePanAngle/360)*360)+doublePanAngle;
}
void updateTargetTiltAngle(){
  tiltAngle=int(atan2((dEle), distance)*radToDeg);
  if(tiltAngle<-45) tiltAngle=-45;
  tiltAngle = map(tiltAngle, -90, 90, 12,120); 
  
}

void maintenanceMode(){
  panServo.writeMicroseconds(1500);
  tiltServo.write(175);  
}

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
  Serial.println("hello world");
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
  updatePanMillis = millis();
  
  //HMI Pins and LEDS
  pinMode(calPin1,INPUT);
  pinMode(calPin2,INPUT);
  pinMode(calPin3,INPUT);

  //Signal Processing
  memset(winLon,0,sizeof(winLon));//fill winLon with zeros
  memset(winLat,0,sizeof(winLat)); //fill winLat with zeros
  Serial.println("HELLO");
}

void loop() {
  recvSerialData(Serial1); //Receieve Data from HC12 Receiver
  if(newData == true){
    parseGPSData();
    winLatAndLon(); //feed GPS data to regression buffer and Moving Average buffer
    newData = false;
  }
  
  recvSerialData(Serial); //Recieve Data from CV program
  if(newData == true){
    parseCVData();
    newData = false;
  } 
     
  if(digitalRead(calPin4) == HIGH) {
	  maintenanceMode();
  }
  else if(digitalRead(calPin1)==HIGH) {
	  calibration();
  }
  else if(digitalRead(calPin3)==HIGH){   
    updateTranLoc();
  
    //Determine Pan Angle
    getPanAngle();

    //Update Target Angles
    if(millis()-updatePanMillis > 1500) {
      updateTargetPanAngle();
      updatePanMillis = millis(); 
    }
    else updateTargetPanAngleCV();
    
    panPID.Compute();
 
    //Servo Control
    panServo.writeMicroseconds((int)outputPanServo);
    
    //Display
    Serial.print(millis()-millisStart);  Serial.print(" ");
    Serial.print("targetPanAngle: "); Serial.print(targetPanAngle);  Serial.print(" ");
    Serial.print("panAngle: "); Serial.print(panAngle);  Serial.print(" ");
    Serial.print("panServoOutput: "); Serial.println(outputPanServo);
  

  }//else if
  else{
  //PID Testing
   getPanAngle(); 
    //PID TESTING
   if((millis()-targetAngleINCMillis) >= 80){
     targetPanAngle += random(-5,5);
     targetAngleINCMillis = millis();
     }
    panPID.Compute();

     
   panServo.writeMicroseconds((int)outputPanServo);

   //Display
   Serial.print(millis()-millisStart);  Serial.print(" ");
   Serial.print("targetPanAngle: "); Serial.print(targetPanAngle);  Serial.print(" ");
   Serial.print("panAngle: "); Serial.print(panAngle);  Serial.print(" ");
   Serial.print("panServoOutput: "); Serial.println(outputPanServo);
   
  }
}//loop

