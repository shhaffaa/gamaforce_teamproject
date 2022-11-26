#include <Wire.h>
#include <Servo.h>


Servo right_prop;
Servo left_prop;

/*MPU-6050 gives you 16 bits data so you have to create some 16int constants
 * to store the data for accelerations and gyro*/

//int16_t Acc_rawX, Acc_rawY, Acc_rawZ,Gyr_rawX, Gyr_rawY, Gyr_rawZ;
 
float Acceleration_angle[2];
float Gyro_angle[2];
float Total_angle[2];

#define MPU 0x68
#define ACCEL 0x3B
#define GYRO 0x43
float temp;
float AccRawX1, AccRawY1, AccRawZ1, GyRawX1, GyRawY1, GyRawZ1;
float AccRawX2, AccRawY2, AccRawZ2, GyRawX2, GyRawY2, GyRawZ2;
float AccAngleX, AccAngleY, AccAngleZ, GyAngleX, GyAngleY, GyAngleZ;
float roll, pitch, yaw;
float mpuTimer, mpuDTime;
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

float PID, pwmLeft, pwmRight, error, previous_error;
float pid_p=0;
float pid_i=0;
float pid_d=0;
/////////////////PID CONSTANTS/////////////////
double kp=4.0585;//3.55
double ki=0.0025;//0.003
double kd=1.5;//2.05
///////////////////////////////////////////////

double throttle_left=1375;
double throttle_right=1300;//initial value of throttle to the motors
float desired_angle = 0; //This is the angle in which we whant the
                         //balance to stay steady


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

void read_mpu6050_accel(){ //ditambahin serial print
  Wire.beginTransmission(MPU);
  Wire.write(ACCEL);
  Wire.endTransmission();
  Wire.requestFrom(MPU, 8);
  AccRawX1 = Wire.read() << 8 | Wire.read();//high and low accel data x
  AccRawY1 = Wire.read() << 8 | Wire.read();//high and low accel data y
  AccRawZ1 = Wire.read() << 8 | Wire.read();//high and low accel data z
//  temp = Wire.read() << 8 | Wire.read();

//  Serial.print("Acc X: ");
//  Serial.print(AccRawX1);
//  Serial.print("Acc Y: ");
//  Serial.print(AccRawY1);
//  Serial.print("Acc Z: ");
//  Serial.print(AccRawZ1);
//  Serial.println(" ");
}

void read_mpu6050_gyro(){ //ditambahin serial print
  Wire.beginTransmission(MPU);
  Wire.write(GYRO);
  Wire.endTransmission();
  Wire.requestFrom(MPU, 6);
  GyRawX1 = Wire.read() << 8 | Wire.read();
  GyRawY1 = Wire.read() << 8 | Wire.read();
  GyRawZ1 = Wire.read() << 8 | Wire.read();

//  Serial.print("Gy X: ");
//  Serial.print(GyRawX1);
//  Serial.print("Gy Y: ");
//  Serial.print(GyRawY1);
//  Serial.print("Gy Z: ");
//  Serial.print(GyRawZ1);
//  Serial.println(" ");
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

//void calculate_angle(){
//  read_mpu6050_accel();
//
//  AccRawX2 = (AccRawX1 + (-1*AccRawXc)) / 8192.0;
//  AccRawY2 = (AccRawY1 + (-1*AccRawYc)) / 8192.0; 
//  AccRawZ2 = (AccRawZ1 + (8192 - AccRawZc)) / 8192.0;
//
//  AccAngleX = (atan2(AccRawX2, AccRawZ2))*57.2957795;
//  AccAngleY = (atan2(AccRawY2, AccRawZ2))*57.2957795;
//
//  for (int i = 0; i < 250; i++){
//    mpuDTime = (millis() - mpuTimer) / 1000;
//    read_mpu6050_gyro();
//    
//    GyRawX2 = GyRawX1 / 65.5;
//    GyRawY2 = GyRawY1 / 65.5;
////    GyRawZ2 = (GyRawZ1 + (-1 * GyRawXc)) / 65.5;
//
//    GyAngleX += GyRawX2 * mpuDTime;
//    GyAngleY += GyRawY2 * mpuDTime;
//  }
//  GyAngleX = (GyAngleX + (-1 * GyRawXc)) / 250;
//  GyAngleY = (GyAngleY + (-1 * GyRawYc)) / 250;
//  
//  roll = 0.96 * AccAngleX + 0.02 * GyAngleX;
//  pitch = 0.96 * AccAngleY + 0.02 * GyAngleY;
//}

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
  /*In order to start up the ESCs we have to send a min value
   * of PWM to them before connecting the battery. Otherwise
   * the ESCs won't start up or enter in the configure mode.
   * The min value is 1000us and max is 2000us, REMEMBER!*/

  left_prop.writeMicroseconds(2000); 
  delay(1000);
  left_prop.writeMicroseconds(1000); 
  
  right_prop.writeMicroseconds(2000);
  delay(1000); 
  right_prop.writeMicroseconds(1000);
  calibrate_mpu6050();
  delay(1000); /*Give some delay, 7s, to have time to connect
                *the propellers and let everything start up*/ 
  
}//end of setup void

void loop() {
/////////////////////////////I M U/////////////////////////////////////
    timePrev = time;  // the previous time is stored before the actual time read
    time = millis();  // actual time read
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
   
   /*Now we have our angles in degree and values from -10º0 to 100º aprox*/
    //Serial.println(Total_angle[1]);

  kalmanR = kalman_filter(Total_angle[0]);
  kalmanP = kalman_filter(pitch);
  //kalmanP = kalman_filter(Total_angle[1]);

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

/*Finnaly using the servo function we create the PWM pulses with the calculated
width for each pulse*/
left_prop.writeMicroseconds(pwmLeft);
right_prop.writeMicroseconds(pwmRight);
previous_error = error; //Rememtber to store the previous error.

  Serial.print("Pitch: ");
  Serial.print(Total_angle[0]);
//  Serial.print(" kalmanP: ");
//  Serial.print(kalmanP);
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
