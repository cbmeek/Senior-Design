/* Camera System Control 
 *  
 *Credit to: 
 *Robin2 - "Serial Input Basics updated" available: http://forum.arduino.cc/index.php?topic=396450
 */
#include <Servo.h>
#include <PID_v1.h>

//Constants
  const int calPin1 = 4;
  const int calPin2 = 5;

  const int panFeedBackPin = 2;
  const int panPin = 8;

  const int tiltPin = 9;
  const int zoomPin = 10;

//holds calibration/initialization data
  float lat0;
  float lon0;
  int ele0;
  
//holds GPS transmistter data
  float lat1;
  float lon1;
  int ele1;
  float latRaw, lonRaw;
  int eleRaw;
  
//holds Pan angle data
  int newQuadrant = 1, oldQuadrant = 1;
  long panAngle=0;
  double doublePanAngle=0;
  unsigned long durationLow = 0, durationHigh = 0;
  long rotation = 0;
  unsigned int dutyCycle = 0, theta = 0;
  
//Pan Control data
  const double outputMax = 1620;
  const double outputMin = 1360;
  const int timeStep=40;   
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
  
void setup() {
  //Pan Servo Setup
  panServo.attach(panPin);
  pinMode(panFeedBackPin, INPUT);
  
  //Tilt Servo Setup
  tiltServo.attach(tiltPin);
  
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
  
}

void loop() {
  
  //Receieve GPS Transmitter Data
  recvSerialData(Serial1); 
  if(newData == true) {
    parseGPSData();
    updateTranLoc();    
  }
  

  //Determine Pan Angle
    getPanAngle();
  
  //Pan Angle PID
  panPID.Compute();
  if(doublePanAngle>=(targetPanAngle-1) && doublePanAngle<=(targetPanAngle+1))
  {
    outputPanServo=1500;
  }

  //Servo Control
  panServo.writeMicroseconds((int)outputPanServo);

  
  //Display
  /*
  Serial1.print(millis()-millisStart);
  Serial1.print(" ");
  Serial1.print(targetPanAngle);
  Serial1.print(" ");
  Serial1.print(panAngle);
  Serial1.print(" ");
  Serial1.println((int)outputPanServo,DEC);
  */
 //PID TESTING
 /* if((millis()-targetAngleINCMillis) >= 1000)
    {
    targetPanAngle += 10;
    targetAngleINCMillis = millis();
  }*/
}

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
}

void updateTranLoc(){
  latRaw = gpsLat;
  lonRaw = gpsLon;
  eleRaw = gpsEle;  
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
    
  panAngle = 360*rotation + theta;
  doublePanAngle = (double)panAngle; //cast the pan angle to double for pid
 /*
 //Print the Data 
  Serial1.print("newQuadrant: ");
  Serial1.print(newQuadrant);
  Serial1.print("\t");
  Serial1.print("oldQuadrant: ");
  Serial1.print(oldQuadrant);
  Serial1.print("\t");
  Serial1.print("Theta: ");  
  Serial1.print(theta,DEC);
  Serial1.print("\t");
  Serial1.print("Rotation #: ");
  Serial1.print(rotation);
  Serial1.print("\t");
  Serial1.print("panAngle: ");
  Serial1.println(panAngle,DEC);
 */ 
}

//Calibrate camera system
//void calibration(float latRaw, float lonRaw, float eleRaw, float &lat0, float &lon0, float &ele0, Servo &tiltServo)
void calibration()
{   
  tiltServo.writeMicroseconds(1500);
  //Code to tell pan angle servo to Home
  
   //SET THE LONGITUDE,LATITUDE, AND ELEVATION OF THE CAMERA SYSTEM
   if(digitalRead(calPin1)== HIGH && digitalRead(calPin2) == HIGH){
    lat0 = latRaw;
    lon0 = lonRaw;
    ele0 = eleRaw;
   }
   
   //SET TRIPOD TO ALIGN WITH GPS TRANSMITTER AND CALCULATE INITIAL POSITION VECTOR
   else if(digitalRead(calPin1)== HIGH && digitalRead(calPin2) == LOW){
    lat1 = latRaw;
    lon1 = lonRaw;
    
    //Do not want to adjust camera tilt away from level. 
    //When aligning camera lens ensure GPS transmistter is centered about Y axis
    ele1 = ele0; 
   } 
}
