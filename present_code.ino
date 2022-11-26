#include <Wire.h>
#include <Servo.h>

Servo right_prop;
Servo left_prop;

#define MPU 0x68
#define ACCEL 0x3B
#define GYRO 0x43

float Acceleration_angle[2];
float Gyro_angle[2];
float Total_angle[2];
float AccRawX1, AccRawY1, AccRawZ1, GyRawX1, GyRawY1, GyRawZ1;

//--calibration
float AccRawXc, AccRawYc, AccRawZc, GyRawXc, GyRawYc, GyRawZc;
float elapsedTime, time, timePrev;
int i;
float rad_to_deg = 180/3.141592654;

/////kalman
float kalmanR;
float kalmanP;
float Xt, Xt_update, Xt_prev;
float Pt, Pt_update, Pt_prev;
float Kt;
float R, Q;

//PID
float PID, pwmLeft, pwmRight, error, previous_error;
float pid_p=0;
float pid_i=0;
float pid_d=0;

/////////////////PID CONSTANTS/////////////////
double kp=4.0585;
double ki=0.0025;
double kd=1.5;
///////////////////////////////////////////////

double throttle_left=1375;
double throttle_right=1300;
float desired_angle = 0;

float kalman_filter(float data){
  Xt_update = Xt_prev;
  Pt_update = Pt_prev + Q;
  Kt = Pt_update / (Pt_update + R);
  Xt = Xt_update + (Kt*(data - Xt_update));
  Pt = (1-Kt)*Pt_update;

  Xt_prev = Xt;
  Pt_prev = Pt; 

  return Xt;
}

void mpu6050_setup(){
  Wire.beginTransmission(MPU); //mpu6050 register
  Wire.write(MPU);
  Wire.write(0b00000111);
  Wire.endTransmission();
  //mpu reset
  Wire.beginTransmission(0x68);
  Wire.write(MPU);
  Wire.write(0b00000000);
  Wire.endTransmission();
 // low pass filter
  Wire.beginTransmission(0x68);
  Wire.write(0x1A);
  Wire.write(0b00000001);
  Wire.endTransmission();
 // powe management / reset 
  Wire.beginTransmission(MPU);
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission();
  // acelero dan high pass filter
  Wire.beginTransmission(MPU);
  Wire.write(0x1C);
  Wire.write(0b00000111);
  Wire.endTransmission();
// gyro reset
  Wire.beginTransmission(MPU);
  Wire.write(0x1B);
  Wire.write(0x00);
  Wire.endTransmission();
}

void read_mpu6050_accel(){
  Wire.beginTransmission(MPU);
  Wire.write(ACCEL);
  Wire.endTransmission();
  Wire.requestFrom(MPU, 8);
  AccRawX1 = Wire.read() << 8 | Wire.read();//high and low accel data x
  AccRawY1 = Wire.read() << 8 | Wire.read();//high and low accel data y
  AccRawZ1 = Wire.read() << 8 | Wire.read();//high and low accel data z
}

void read_mpu6050_gyro(){
  Wire.beginTransmission(MPU);
  Wire.write(GYRO);
  Wire.endTransmission();
  Wire.requestFrom(MPU, 6);
  GyRawX1 = Wire.read() << 8 | Wire.read();
  GyRawY1 = Wire.read() << 8 | Wire.read();
  GyRawZ1 = Wire.read() << 8 | Wire.read();
}

void calibrate_mpu6050(){
  Serial.print("calibrating");

  for (int i = 0; i < 2000; i++){
    read_mpu6050_accel();
    read_mpu6050_gyro();
    AccRawXc += AccRawX1;
    AccRawYc += AccRawY1;
    AccRawZc += AccRawZ1;
    GyRawXc += GyRawX1;
    GyRawYc += GyRawY1;
    GyRawZc += GyRawZ1;
    
    if (i % 100 == 0) Serial.print(".");
  }
  AccRawXc /= 2000;
  AccRawYc /= 2000;
  AccRawZc /= 2000;
  GyRawXc /= 2000;
  GyRawYc /= 2000;
  GyRawZc /= 2000;
  Serial.println(AccRawXc);
  Serial.println(AccRawYc);
  Serial.println(AccRawZc);
}

void setup() {

  R = 10;
  Q = 0.1;
  Pt_prev = 1;
  
  Wire.begin(); //begin the wire comunication
  
  Serial.begin(115200);
  right_prop.attach(3); //attatch the right motor to pin 3
  left_prop.attach(5);  //attatch the left motor to pin 5
  mpu6050_setup();
  time = millis(); //Start counting time in milliseconds

  left_prop.writeMicroseconds(2000); 
  delay(1000);
  left_prop.writeMicroseconds(1000); 
  
  right_prop.writeMicroseconds(2000);
  delay(1000); 
  right_prop.writeMicroseconds(1000);
  calibrate_mpu6050();
  delay(1000);
  
}//end of setup void

void loop() {
/////////////////////////////I M U/////////////////////////////////////
   timePrev = time;  
   time = millis();  
   elapsedTime = (time - timePrev) / 1000; 

   read_mpu6050_accel();
   /*---X---*/
   Acceleration_angle[0] = atan((AccRawY1/16384.0)/sqrt(pow((AccRawX1/16384.0),2) + pow((AccRawZ1/16384.0),2)))*rad_to_deg;
   /*---Y---*/
   Acceleration_angle[1] = atan(-1*(AccRawX1/16384.0)/sqrt(pow((AccRawY1/16384.0),2) + pow((AccRawZ1/16384.0),2)))*rad_to_deg;

   read_mpu6050_gyro();
   /*---X---*/
   Gyro_angle[0] = GyRawX1/131.0; 
   /*---Y---*/
   Gyro_angle[1] = GyRawY1/131.0;

   /*---X axis angle---*/
   Total_angle[0] = 0.98 *(Total_angle[0] + Gyro_angle[0]*elapsedTime) + 0.02*Acceleration_angle[0];
   /*---Y axis angle---*/
   Total_angle[1] = 0.98 *(Total_angle[1] + Gyro_angle[1]*elapsedTime) + 0.02*Acceleration_angle[1];
   
  kalmanR = kalman_filter(Total_angle[0]);
  kalmanP = kalman_filter(Total_angle[1]);

error = Total_angle[0] - desired_angle;

pid_p = kp*error;
  if(-5 <error <5)
  {
    pid_i = pid_i+(ki*error);  
  }
pid_d = kd*((error - previous_error)/elapsedTime);
PID = pid_p + pid_i + pid_d;
  if(PID < -1000)
  {
    PID=-1000;
  }
  if(PID > 1000)
  {
    PID=1000;
  }

pwmLeft = throttle_left - PID + 75;
pwmRight = throttle_right + PID;

//threshold
//Right
if(pwmRight < 1100)
{
  pwmRight= 1100;
}
if(pwmRight > 2000)
{
  pwmRight=2000;
}
//Left
if(pwmLeft < 1100)
{
  pwmLeft= 1100;
}
if(pwmLeft > 1800)
{
  pwmLeft=1800;
}

left_prop.writeMicroseconds(pwmLeft);
right_prop.writeMicroseconds(pwmRight);
previous_error = error; //Remember to store the previous error.

  Serial.print("Pitch: ");
  Serial.print(Total_angle[0]);
  Serial.print(" kalmanP: ");
  Serial.print(kalmanP);
  Serial.print(" error: ");
  Serial.print(error);
  Serial.print(" ");

  Serial.print("Right: ");
  Serial.print(pwmRight);
  Serial.print(" Left: ");
  Serial.print(pwmLeft); 
  Serial.print(" PID: ");
  Serial.print(PID);
  Serial.println(" ");
  delay(10);

}//end of loop void
