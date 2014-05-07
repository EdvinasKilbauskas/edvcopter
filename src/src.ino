
#include <Wire.h>
#include "MPUSensor.h"
#include <Servo.h>
#include "PID_v1.h"
#include "RCReceiver.h"

//#define DEBUG

#define ESC_A 5 
#define ESC_B 3
#define ESC_C 9
#define ESC_D 6
#define RC1_PIN 4
#define RC2_PIN 7
#define RC3_PIN 8
#define RC4_PIN 12
#define RC5_PIN 13
#define RC6_PIN 11
#define RC_POWER_PIN A0

#define ESC_MIN 25
#define ESC_MAX 120

#define PITCH_P 0.201f
#define PITCH_I 0.040f
#define PITCH_D 0.108f 
#define PITCH_MAX_MOTOR_BALANCE_SPEED 7                 // max amount of thrust that will be applied to balance this axis
#define PITCH_PID_OUTPUT 20
#define PITCH_ERROR_CORRECTION -0.010

#define ROLL_P 0.201f
#define ROLL_I 0.040f
#define ROLL_D 0.108f
#define ROLL_MAX_MOTOR_BALANCE_SPEED 7                  // max amount of thrust that will be applied to balance this axis
#define ROLL_PID_OUTPUT 20
#define ROLL_ERROR_CORRECTION 0

#define YAW_P 0.3f
#define YAW_I 0.0f
#define YAW_D 0.06f
#define YAW_PID_OUTPUT 20
#define YAW_MAX_MOTOR_BALANCE_SPEED 15                   // max amount of thrust that will be applied to balance this axis
#define YAW_ERROR_CORRECTION 0.0248f 

#define MAX_YAW_SPEED 90                                 // angle in degrees of rotation per second when controlled with RC controler
#define MAX_ROLL_ANGLE 30
#define MAX_PITCH_ANGLE 30

#define HOVER_MOTOR_SPEED 10

#define ESC_ARM_TIME 1000                                // in milliseconds, this requires so much time because I also need to wait for sensor values to normalize.
#define SENSOR_NORMALIZE_TIME 1000*20

double pitchSp, rollSp, yawSp = 0;                       // setpoints
bool yawSpSet = false;
double P,I,D;                                            // PID values 
float velocity;                                          // global velocity
double bal_ac = 0, bal_bd, bal_axes = 0;                 // motor balances can vary between -100 & 100, motor balance between axes -100:ac , +100:bd
float deltaTime = 0;

double va, vb, vc, vd, v_ac, v_bd = 0;                   // velocities
double pitch, roll, yaw  = 0.0;                          // angles in degrees

Servo a,b,c,d;                                           // motors

PID pitchReg(&pitch, &bal_bd, &pitchSp, PITCH_P, PITCH_I, PITCH_D, DIRECT);
PID rollReg(&roll, &bal_ac, &rollSp, ROLL_P, ROLL_I, ROLL_D, DIRECT);
PID yawReg(&yaw, &bal_axes, &yawSp, YAW_P, YAW_I, YAW_D, DIRECT);

 
RCReceiver rc;
MPUSensor sensor;

void setup(){
  
  pinMode(A1,OUTPUT);
  pinMode(A2,OUTPUT);
  pinMode(A3,OUTPUT);
  
  digitalWrite(A1,LOW);
  digitalWrite(A2,LOW);
  digitalWrite(A3,LOW);
  sensor.init();
  rc.init(RC1_PIN,RC2_PIN,RC3_PIN,RC4_PIN,RC5_PIN,RC6_PIN,RC_POWER_PIN);
  
  #ifndef DEBUG
  delay(SENSOR_NORMALIZE_TIME);
  #endif
  
  initPIDs();
  initESCs();
  armESCs();
  
  #ifdef DEBUG                        // Device tests go here
  
  Serial.begin(115200);                 // Serial only necessary if in DEBUG mode
  Serial.flush();
  log("All system initialized at: ", millis()/1000.0f);
  Serial.println("All system initialized and ready");
  #endif
  
}

void loop(){ 
   long start = micros();
   
   sensor.waitTillReady();

   configPID();
   computeRCValues();
   computeRotation();
   computeVelocities();
   
   if(yaw != 180 & yaw != 0 && yawSpSet == false){
      yawSp = yaw;
      yawSpSet = true; 
   }
   

   //keepMotorsRunning();
   runSingleMotor();
   updateMotors();
   logging();
   
   sensor.calculate();
   
   deltaTime = (micros()-start)/1000000.0f;
   
   
}

void runSingleMotor(){
   va = vb = vc = vd = velocity;
   
   float rc6 = rc.getChannel(6);
   
   if(rc6 > 0.75){
     vb = vc = vd = 0;
     
   }else if(rc6 > 0.5f){
     va = vc = vd = 0;
   }else if(rc6 > 0.25f){
     va = vb = vd = 0;
   }else if(rc6 > 0.0f){
     va = vb = vc = 0;
   }else{
      va = vb = vc = vd = 0; 
   }
}

void keepMotorsRunning(){
  if(va < 35){
     va = 35; 
  }
  if(vb < 40){
     vb = 40; 
  }
  if(vc < 35){
     vc = 35; 
  }
  if(vd < 40){
     vd = 40; 
  }
}


void computeRCValues()
{
   velocity = rc.getChannel(3)*ESC_MAX;
   if(velocity < ESC_MIN) velocity = ESC_MIN;
   
   float sp = rc.getChannel(4);    // abbreviation 'sp' (yawSp, rollSp, pitchSp) stands for set point, this is the angle at which we WANT to be at. PID algorithm will try to change motor speeds accordingaly to keep the actual angle of the quadcopter in level with our desired (set point) angle 
   if(abs(sp-0.5f) >= 0.05f){
     yawSp += (sp-0.5f)*MAX_YAW_SPEED*2*deltaTime;
   }

   rollSp = 0;
   pitchSp = 0;
   
   sp = rc.getChannel(2);
   
   if(abs(sp-0.5f) >= 0.05f){
      rollSp += (sp-0.5f)*MAX_ROLL_ANGLE*3;
      pitchSp += (sp-0.5f)*MAX_ROLL_ANGLE*3;
   }
   
   sp = rc.getChannel(1);
   
   if(abs(sp-0.5f) >= 0.05f){
      rollSp -= (sp-0.5f)*MAX_PITCH_ANGLE*3;
      pitchSp += (sp-0.5f)*MAX_PITCH_ANGLE*3;
   }
   
   //setpoint = (rc.getChannel(1)-0.5f)*20.0f;
}

void computeRotation()
{
   pitch = ((sensor.getPitch()+PITCH_ERROR_CORRECTION)*(180/M_PI)); // Get value from sensor, correct it, and convert from radians to degrees
   roll = ((sensor.getRoll()+ROLL_ERROR_CORRECTION)*(180/M_PI));    // Same thing here
   yaw = ((sensor.getYaw()+ROLL_ERROR_CORRECTION)*(180/M_PI));
}

void computeVelocities()
{
    if(pitchReg.Compute()){
     bal_bd /= PITCH_PID_OUTPUT;
     vd = velocity+(bal_bd*PITCH_MAX_MOTOR_BALANCE_SPEED);
     vb = velocity-(bal_bd*PITCH_MAX_MOTOR_BALANCE_SPEED);
   }
   
   if(rollReg.Compute()){
     bal_ac /= ROLL_PID_OUTPUT;
     va = velocity+(bal_ac*ROLL_MAX_MOTOR_BALANCE_SPEED);
     vc = velocity-(bal_ac*ROLL_MAX_MOTOR_BALANCE_SPEED);
   }
   
   if(yawReg.Compute()){
      bal_axes /= YAW_PID_OUTPUT;
      va -= bal_axes*YAW_MAX_MOTOR_BALANCE_SPEED;
      vc -= bal_axes*YAW_MAX_MOTOR_BALANCE_SPEED;
      
      vb += bal_axes*YAW_MAX_MOTOR_BALANCE_SPEED;
      vd += bal_axes*YAW_MAX_MOTOR_BALANCE_SPEED;
   }
   
   //vb = 0;
}

// call this when you want to control PID values dirrectly while flying.
void configPID()
{
    P = rc.getChannel(6)*0.5f;
    D = rc.getChannel(5)*0.3f;
    //D = rc.getChannel(5)*0.2f;;//rc.getChannel(5)*0.25f;
    I = ROLL_I;
    rollReg.SetTunings(P,ROLL_I,D);
    pitchReg.SetTunings(P,PITCH_I,D);
}

void updateMotors(){
  a.write(va);
  c.write(vc);
  b.write(vb);
  d.write(vd);
}

void initESCs(){
  // attach ESCs to servos
  a.attach(ESC_A);
  b.attach(ESC_B);
  c.attach(ESC_C);
  d.attach(ESC_D);
}

void armESCs(){
  va = vb = vc = vd = ESC_MIN;
  updateMotors();
  delay(ESC_ARM_TIME);
}

void initPIDs(){
  pitchReg.SetMode(AUTOMATIC);
  pitchReg.SetOutputLimits(-PITCH_PID_OUTPUT, PITCH_PID_OUTPUT);
  pitchReg.SetSampleTime(14);
  
  rollReg.SetMode(AUTOMATIC);
  rollReg.SetOutputLimits(-ROLL_PID_OUTPUT, ROLL_PID_OUTPUT);
  rollReg.SetSampleTime(14);
  
  yawReg.SetMode(AUTOMATIC);
  yawReg.SetOutputLimits(-YAW_PID_OUTPUT, YAW_PID_OUTPUT);
  yawReg.SetSampleTime(14);
}

void log(const char * str, float val)
{
   Serial.print(str);
   Serial.print(val,3); 
}

void logging()
{
  #ifdef DEBUG

  //logSetpoints();
  //logYPRDeg();
  //logMotorVel();
  logPID();
  
  newLine();
   
  #endif
}

void newLine()
{
  #ifdef DEBUG
    Serial.println();
  #endif
}

void logSetpoints()
{
   log("\tPitchSP: ", pitchSp);
   log("\tRollSP: ", rollSp);
   log("\tYawSP: ", yawSp);
}

void logYPRDeg(){
   log("\tPitch: ", pitch);
   log("\tRoll: ", roll);
   log("\tYaw: ", yaw);
}

void logPID(){
   Serial.print("\tP val: "); Serial.print(P,3);
   Serial.print("\tI val: "); Serial.print(I,3);
   Serial.print("\tD val: "); Serial.print(D,3);
}

void logMotorVel()
{
  #ifdef DEBUG
  Serial.print("\tva: "); Serial.print(va);
  Serial.print("\tvc: "); Serial.print(vc);
  Serial.print("\tvb: "); Serial.print(vb);
  Serial.print("\tvd: "); Serial.print(vd);
  #endif
}






