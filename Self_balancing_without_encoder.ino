
#include "TimerOne.h"
#include<Wire.h>

const int MPU_addr=0x68;
double AcX,AcY,AcZ,Tmp,GyX,GyY,GyZ; //These will be the raw data from the MPU6050.
uint32_t timer; //it's a timer, saved as a big-ass unsigned int.  We use it to save times from the "micros()" command and subtract the present time in microseconds from the time stored in timer to calculate the time for each loop.
double compAngleX, compAngleY; //These are the angles in the complementary filter
#define degconvert 57.2957786 //there are like 57 degrees in a radian.




//const byte interruptEncoderPinA = 2;
//const byte interruptEncoderPinB = 3;
//const byte interruptEncoderPinC = 18;
//const byte interruptEncoderPinD = 19;
const byte PWMOutputMotorA=4;
const byte PWMOutputMotorB=5;
const byte InputAA=7;
const byte InputAB=8;
const byte InputBA=9;
const byte InputBB=10;
//volatile int ext_counterA=0;
//volatile int ext_counterB=0;
volatile float rpm1=0.0;
volatile float rpm2=0.0;
volatile bool direction_rotation=true;


const float theta_kp=25.0;
const float theta_ki=0.0;
const float theta_kd=0.02;

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
volatile float PID_e;




double getAngle(){

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
  
  double dt = (double)(micros() - timer) / 1000000; //This line does three things: 1) stops the timer, 2)converts the timer's output to seconds from microseconds, 3)casts the value as a double saved to "dt".
  timer = micros(); //start the timer again so that we can calculate the next dt.

//  double roll = atan2(AcY, AcZ)*degconvert;
  double pitch = atan2(-AcX, AcZ)*degconvert;


//  double gyroXrate = GyX/131.0;
  double gyroYrate = GyY/131.0;


 // compAngleX = 0.99 * (compAngleX + gyroXrate * dt) + 0.01 * roll; // Calculate the angle using a Complimentary filter
  compAngleY = 0.99 * (compAngleY + gyroYrate * dt) + 0.01 * pitch; 
   return compAngleY;
//  //W00T print dat shit out, yo!
//  Serial.print(compAngleX);Serial.print("\t");
//  Serial.print(compAngleY);Serial.print("\n");
}


void setup() {
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

  //setup starting angle
  //1) collect the data
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

  //2) calculate pitch and roll
  double roll = atan2(AcY, AcZ)*degconvert;
  double pitch = atan2(-AcX, AcZ)*degconvert;

  //3) set the starting angle to this pitch and roll
  double gyroXangle = roll;
  double gyroYangle = pitch;
  double compAngleX = roll;
  double compAngleY = pitch;

  //start a timer
  timer = micros();
}

void loop() {

curr_theta = getAngle() / degconvert;
Serial.print("getangle: ");Serial.println(getAngle());
Serial.print("angle: ");Serial.println(curr_theta);
curr_theta_error = target_theta - curr_theta;

theta_sum_error += curr_theta; 

theta_diff_error = curr_theta_error - prev_theta_error;

PID_e = (theta_kp*curr_theta_error)+(theta_ki*theta_sum_error)+(theta_kd*theta_diff_error);

PWM = (int) ( PID_e ) * 1024;

PWM = fabs(PWM);

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
  
 


}
