#define degconvert 57.2957786 

#include "TimerOne.h"
#include<Wire.h>

const int MPU_addr=0x68;
double AcX,AcY,AcZ,Tmp,GyX,GyY,GyZ;
uint32_t timer;
double compAngleX, compAngleY; 


const byte interruptEncoderPinA = 2;
const byte interruptEncoderPinB = 3;
const byte interruptEncoderPinC = 18;
const byte interruptEncoderPinD = 19;
const byte PWMOutputMotorA=4;
const byte PWMOutputMotorB=5;
const byte InputAA=7;
const byte InputAB=8;
const byte InputBA=9;
const byte InputBB=10;
volatile int ext_counterA=0;
volatile int ext_counterB=0;
volatile float rpm1=0.0;
volatile float rpm2=0.0;
volatile bool direction_rotation=true;




//Initialise Ki, Kp, Kd
const float kp=1.0;
const float theta_kp=1.0;
const float theta_ki=0.025;
const float theta_kd=0.005;

//Definition for rpm
const float max_rpm=470.0;
volatile float prev_error=0.0;
volatile float sum_error=0.0;
volatile float diff_error=400.0;
volatile float set_rpm=400.0;
volatile float curr_rpm=0;

//Definitions for angle and acceleration.
const float g=9.81;
volatile float theta=0.0;
volatile float curr_acc=0.0;
volatile float acc_const=0.0025;  // Please set this value according the time of 0.1 ms and radius value = r/delta_t
const float target_theta=0.0;
volatile float curr_theta=0.0;
volatile float theta_sum_error=0.0;
volatile float theta_diff_error=0.0;
volatile float prev_theta_error=0.0;
volatile float req_acc=0.0;
volatile float curr_theta_error;
volatile float curr_error;
volatile int PWM;

double getCurrentAngle(){
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x3B);  // starting with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_addr,14,true); 
  AcX=Wire.read()<<8|Wire.read();  // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)     
  AcY=Wire.read()<<8|Wire.read();  // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
  AcZ=Wire.read()<<8|Wire.read();  // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
  Tmp=Wire.read()<<8|Wire.read();  // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
  GyX=Wire.read()<<8|Wire.read();  // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
  GyY=Wire.read()<<8|Wire.read();  // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
  GyZ=Wire.read()<<8|Wire.read();  // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)
  
  double dt = (double)(micros() - timer) / 1000000;
  timer = micros(); 


  double roll = atan2(AcY, AcZ)*degconvert;
  double pitch = atan2(-AcX, AcZ)*degconvert;

  double gyroXrate = GyX/131.0;
  double gyroYrate = GyY/131.0;
 
  compAngleX = 0.99 * (compAngleX + gyroXrate * dt) + 0.01 * roll; 
  compAngleY = 0.99 * (compAngleY + gyroYrate * dt) + 0.01 * pitch; 
  

  return compAngleY;
}

void setup() {
  // put your setup code here, to run once

  //Set pin input and pulldown it.
  pinMode(interruptEncoderPinA, INPUT);
  digitalWrite(interruptEncoderPinA, LOW);
  pinMode(interruptEncoderPinB, INPUT);
  digitalWrite(interruptEncoderPinB, LOW);
  digitalWrite(InputAA, HIGH);
  digitalWrite(InputBA, HIGH);
  digitalWrite(InputBB, LOW);
  digitalWrite(InputAB, LOW);

  //Initialise the Timer and attach Interrupt.
  Timer1.initialize(50000);
  Timer1.attachInterrupt(calculateRPM);
  
  //Attach Interrupts to the encoders input.
  attachInterrupt(digitalPinToInterrupt(interruptEncoderPinA),increase_CounterA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(interruptEncoderPinB),increase_CounterA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(interruptEncoderPinC),increase_CounterB, CHANGE);
  attachInterrupt(digitalPinToInterrupt(interruptEncoderPinD),increase_CounterB, CHANGE);

    // Set up MPU 6050:
  Wire.begin();
  #if ARDUINO >= 157
  Wire.setClock(400000UL); // Set I2C frequency to 400kHz
  #else
    TWBR = ((F_CPU / 400000UL) - 16) / 2; // Set I2C frequency to 400kHz
  #endif

  
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x6B);  // PWR_MGMT_1 register
  Wire.write(0);     // set to zero (wakes up the MPU-6050)
  Wire.endTransmission(true);
  Serial.begin(115200);
  delay(100);

  Wire.beginTransmission(MPU_addr);
  Wire.write(0x3B);  // starting with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_addr,14,true);  // request a total of 14 registers
  AcX=Wire.read()<<8|Wire.read();  // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)     
  AcY=Wire.read()<<8|Wire.read();  // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
  AcZ=Wire.read()<<8|Wire.read();  // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
  Tmp=Wire.read()<<8|Wire.read();  // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
  GyX=Wire.read()<<8|Wire.read();  // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
  GyY=Wire.read()<<8|Wire.read();  // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
  GyZ=Wire.read()<<8|Wire.read();  // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)

  double roll = atan2(AcY, AcZ)*degconvert;
  double pitch = atan2(-AcX, AcZ)*degconvert;

  double gyroXangle = roll;
  double gyroYangle = pitch;
  double compAngleX = roll;
  double compAngleY = pitch;

  timer = micros();
}

void loop() {
  
  //Set PID on theta.
  curr_theta=getCurrentAngle()/degconvert;  // Get it from some device. Change it later. And after you do, remove this comment.
  curr_theta_error=target_theta - curr_theta;
  theta_sum_error += curr_theta_error;
  if(theta_sum_error > 1.57)
    theta_sum_error =1.57;
  else if(theta_sum_error< -1.57)
    theta_sum_error = -1.57;
 
  theta_diff_error = curr_theta_error - prev_theta_error ;

  //Apply PID to determine the required acceleration.
  req_acc=((theta_kp*curr_theta_error)+(theta_ki*theta_sum_error)+(theta_kd*theta_diff_error))*g;

  //Update the theta_error value
  prev_theta_error=curr_theta_error;

  //Set set_rpm on the basis of required acceleration.
  set_rpm=(req_acc/0.002)+curr_rpm;

  //Calculate errors and other terms
  curr_error= set_rpm - curr_rpm;

  //Apply PWM accordingly.
  PWM=(int)((set_rpm+(kp*curr_error))/max_rpm)*1024;
  PWM = fabs(PWM);
  
 // go();
  
  if(compAngleY <0){
    direction_rotation=false;
    if(PWM<1024){
      digitalWrite(InputAA, HIGH);
      digitalWrite(InputBA, HIGH);
      digitalWrite(InputBB, LOW);
      digitalWrite(InputAB, LOW);
      Timer1.pwm(PWMOutputMotorA, PWM);
      Timer1.pwm(PWMOutputMotorB, PWM);
//      Serial.print(PWM);Serial.print("\n");
    }
    else{
      digitalWrite(InputAA, HIGH);
      digitalWrite(InputBA, HIGH);
      digitalWrite(InputBB, LOW);
      digitalWrite(InputAB, LOW);
      Timer1.pwm(PWMOutputMotorA, 1024);
      Timer1.pwm(PWMOutputMotorB, 1024);
//      Serial.print(PWM);Serial.print("\n");
    }
  }
  else{
    direction_rotation=true;
    if(PWM >(-1)*1024){
      digitalWrite(InputAB, HIGH);
      digitalWrite(InputBB, HIGH);
      digitalWrite(InputBA, LOW);
      digitalWrite(InputAA, LOW);
      Timer1.pwm(PWMOutputMotorA, PWM);
      Timer1.pwm(PWMOutputMotorB, PWM);
//      Serial.print(PWM);Serial.print("\n");
    }
    else{
      digitalWrite(InputAA, HIGH);
      digitalWrite(InputBA, HIGH);
      digitalWrite(InputBB, LOW);
      digitalWrite(InputAB, LOW);
      Timer1.pwm(PWMOutputMotorA, 1024);
      Timer1.pwm(PWMOutputMotorB, 1024);
//      Serial.print(PWM);Serial.print("\n");
    }
  }
  
  //Update the error value.
  prev_error= curr_error;
  
}

void increase_CounterA() {
  ext_counterA++;
}

void increase_CounterB(){
  ext_counterB++;
}

void calculateRPM(){
  rpm1=(ext_counterA*60*10)/(64*19);
  if(direction_rotation==false)
    rpm1=(-1)*rpm1;
    
  rpm2=(ext_counterB*60)/(6.4*19);
  if(direction_rotation==false)
    rpm2=(-1)*rpm2;
    
  curr_acc=acc_const * (((rpm1+rpm2)/2)-curr_rpm);
  
  curr_rpm=(rpm1+rpm2)/2;
  
  ext_counterA=0;
  ext_counterB=0;
  
  Serial.print("RPM1 ");Serial.print(rpm1);
  Serial.print(" RPM2 ");Serial.println(rpm2);
  Serial.print("Angle");Serial.println(compAngleY);
}


