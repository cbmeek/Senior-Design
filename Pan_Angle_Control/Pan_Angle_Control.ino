#include <PID_v1.h>;
#include <Servo.h>;
Servo pan_servo;

const int feedbackPin = 2;
unsigned long millisStart,durationLow = 0, durationHigh = 0, millisLast = 0;
double rotation = 0, angle;
unsigned int dutyCycle = 0, theta = 0;
int newQuadrant = 1, oldQuadrant = 1;
double targetAngle=360;
double output=1500;
double Kp=9*.5;
double Ki=1.2*9/2;
double Kd=3*9*2/40;

PID pan_pid(&angle, &output, &targetAngle, Kp, Ki, Kd, DIRECT);

void setup() {
  pan_servo.attach(8);
  pinMode(feedbackPin, INPUT);
  pan_pid.SetMode(AUTOMATIC); //automatically start pid
  pan_pid.SetOutputLimits(1200,1700); //PID value bounds
  pan_pid.SetControllerDirection(REVERSE);
  pan_pid.SetSampleTime(15); //How many milli seconds between pid calculation
  
  Serial.begin(9600);
  while (!Serial) {
  }
  millisStart=millis();
}

void loop() {

  // Calculate Duty Cycle
  durationLow = pulseIn(feedbackPin, LOW); //Measures the time the feedback signal is low
  durationHigh = pulseIn(feedbackPin, HIGH); //Measures the time the feedback signal is high
  dutyCycle = (10000 * durationHigh) / (durationHigh + durationLow); //calculates the duty cycle

  //Formula for angle calculation taken from parallax 360 feedback data sheet
  theta = (35900 * (dutyCycle - 290)) / (9710 - 290 + 1) / 100;
  if (theta > 359) theta = 359; //limits theta to bounds

  //Determines what quadrant the servo rotates from to determine positive and negative angles outside 0-359
  if ( theta >= 0 && theta <= 90) {
    oldQuadrant = newQuadrant;
    newQuadrant = 1;
  }
  if ( theta > 90 && theta <= 180) {
    oldQuadrant = newQuadrant;
    newQuadrant = 2;
  }
  if ( theta > 180 && theta <= 270) {
    oldQuadrant = newQuadrant;
    newQuadrant = 3;
  }
  else if ( theta > 270 && theta <= 359 ) {
    oldQuadrant = newQuadrant;
    newQuadrant = 4;
  }
  if (oldQuadrant == 1 && newQuadrant == 4) {
    rotation = rotation - 1;
  }
  else if ( oldQuadrant == 4 && newQuadrant == 1) {
    rotation = rotation + 1;
  }

  angle = 360 * rotation + theta;
  pan_pid.Compute();
  
  //Print the Data
  Serial.print((millis()-millisStart));
  Serial.print(",");
  Serial.print(output);
  Serial.print(",");
  Serial.println(angle,DEC);
 

  if ((millis() - millisLast) >= 20) {
    pan_servo.writeMicroseconds(output);
    millisLast = millis();
  }

}
