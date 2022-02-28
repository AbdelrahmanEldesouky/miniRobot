#include <Wire.h>

/******** IMU Variables ********/
#define MPU_addr            0x68
#define Rad_to_Deg          (180.0 / M_PI)
#define Deg_to_Rad          (M_PI / 180.0)

#define Samples_Number       100
#define Delay_Calibration    10      

double ax, ay, az;
double gx, gy, gz;

double ax_reg, ay_reg, az_reg;
double gx_reg, gy_reg, gz_reg;

double ax_calibration = 0, ay_calibration = 0, az_calibration = 0; 
double gx_calibration = 0, gy_calibration = 0, gz_calibration = 0;

void IMU_Init(void) ;
void IMU_Update (void) ; 

void IMU_Init(void)
{
    Wire.begin();
    Wire.beginTransmission(MPU_addr);
    Wire.write(0x6B);
    Wire.write(0x00);
    Wire.endTransmission(true);

    //Calibration
    //Take N samples while the robot is at rest.
    for (int i = 0; i < Samples_Number; i++)
    {
        //Accelerometer
        Wire.beginTransmission(MPU_addr);
        Wire.write(0x3B);
        Wire.endTransmission(false);
        Wire.requestFrom(MPU_addr,6,true);
  
        ax_calibration += Wire.read()<<8|Wire.read();    
        ay_calibration += Wire.read()<<8|Wire.read();
        az_calibration += Wire.read()<<8|Wire.read();
  
        //Gyroscope
        Wire.beginTransmission(MPU_addr);
        Wire.write(0x43);
        Wire.endTransmission(false);
        Wire.requestFrom(MPU_addr,6,true);
  
        gx_calibration += Wire.read()<<8|Wire.read();
        gy_calibration += Wire.read()<<8|Wire.read();
        gz_calibration += Wire.read()<<8|Wire.read();
  
        delay(Delay_Calibration);
    }

    //Take the average
    ax_calibration /= Samples_Number, ay_calibration /= Samples_Number, az_calibration /= Samples_Number;
    gx_calibration /= Samples_Number, gy_calibration /= Samples_Number, gz_calibration /= Samples_Number;
    
    //The expected register value when the robot is at rest on a horizontal plane should be 16384(=1g)
    az_calibration -= 16384;  
}

void IMU_Update (void)
{
  // read Acc
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_addr,6,true);
    
  ax_reg = Wire.read()<<8|Wire.read();    
  ay_reg = Wire.read()<<8|Wire.read();
  az_reg = Wire.read()<<8|Wire.read();
  
  //Calibrate the register readings then convert them to (m/s^2)
  ax = ((ax_reg - ax_calibration) / 16384.0) * 9.8;
  ay = ((ay_reg - ay_calibration) / 16384.0) * 9.8;
  az = ((az_reg - az_calibration) / 16384.0) * 9.8;

  // read Gyro
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x43);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_addr,6,true);
    
  gx_reg = Wire.read()<<8|Wire.read();
  gy_reg = Wire.read()<<8|Wire.read();
  gz_reg = Wire.read()<<8|Wire.read();

  //Calibrate the register readings then convert them to (rad/s)
  gx = (((gx_reg - gx_calibration) / 65.5) * Deg_to_Rad) / 2 ;
  gy = (((gy_reg - gy_calibration) / 65.5) * Deg_to_Rad) / 2 ;
  gz = (((gz_reg - gz_calibration) / 65.5) * Deg_to_Rad) / 2 ;  
}
