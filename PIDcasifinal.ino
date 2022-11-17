#include <PID_v1.h>
#include <PIDController.h>
float sensorValue;

//Define Variables we'll be connecting to
double Setpoint, Input, Output;

//Define the aggressive and conservative Tuning Parameters
double aggKp=1, aggKi=0.05, aggKd=2;
double consKp=1, consKi=0.05, consKd=2;

//Specify the links and initial tuning parameters
PID myPID(&Input, &Output, &Setpoint, consKp, consKi, consKd, DIRECT);

void setup(){
  //initialize the variables we're linked to
  Input = analogRead(A1);
  Setpoint = analogRead(A2);
  //turn the PID on
  myPID.SetMode(AUTOMATIC);
  Serial.begin(9600);
}

void loop(){
  Input = analogRead(A2);
  Setpoint = analogRead(A1);
  double gap = abs(Setpoint-Input); //distance away from setpoint
  if(gap < 400){
    //we're close to setpoint, use conservative tuning parameters
    myPID.SetTunings(consKp, consKi, consKd);
  }else{
    //we're far from setpoint, use aggressive tuning parameters
    myPID.SetTunings(aggKp, aggKi, aggKd);
  }
  myPID.SetOutputLimits(51,255);
  myPID.Compute();
  analogWrite(3,Output);
  Serial.println(Input);
  Serial.println(Setpoint);
}
