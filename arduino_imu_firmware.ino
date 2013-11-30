// MPU-6050 Accelerometer + Gyro

//
// Temperature sensor from -40 to +85 degrees Celsius
//   340 per degrees, -512 at 35 degrees.
//
// At power-up, all registers are zero, except these two:
//      Register 0x6B (PWR_MGMT_2) = 0x40  (I read zero).
//      Register 0x75 (WHO_AM_I)   = 0x68.
// 

#include <Wire.h>
#include <SoftwareSerial.h>
#include <math.h>

#define MPU6050_I2C_ADDRESS 0x68

#define FREQ  50.0 // sample freq in Hz

// Bluetooth transmitter 
SoftwareSerial BTSerial(2, 3); // RX | TX

// global angle, gyro derived
 double gx = 0, gy = 0, gz = 0;



void setup()
{      
  int error;
  uint8_t c;
  BTSerial.begin(38400);


  // Initialize the 'Wire' class for the I2C-bus.
  Wire.begin();

  // wake up
  MPU6050_write_reg (0x6b, 0);

  // Low pass filter samples, 1khz sample rate
  MPU6050_write_reg (0x1a, 1);

  // set sample rate
  // sample rate FREQ = 1khz / (div + 1)
  // 1kHz / (div + 1) = FREQ  
  // reg_value = 1khz/FREQ - 1
  // so write 99 to smprt_div (0x19)
  MPU6050_write_reg (0x19, 1000/FREQ - 1);
}


void loop()
{
  int error;
  double dT;
  double ax, ay, az;
 
  uint8_t i2cData[14];
  int16_t accX, accY, accZ;
  double gyrX, gyrY, gyrZ;

  // read imu data
  error = MPU6050_read(0x3b, i2cData, 14);
  
  accX = ((i2cData[0] << 8) | i2cData[1]);
  accY = ((i2cData[2] << 8) | i2cData[3]);
  accZ = ((i2cData[4] << 8) | i2cData[5]);


  gyrX = ((i2cData[8] << 8) | i2cData[9]) / 131.0;
  gyrY = ((i2cData[10] << 8) | i2cData[11]) / 131.0;
  gyrZ = ((i2cData[12] << 8) | i2cData[13]) / 131.0;



  if(error!=0)
    BTSerial.println("ERROR!");

// angles based on accelerometer
  ax = atan2(accX, sqrt( pow(accY, 2) + pow(accZ, 2))) * 180 / M_PI;
  ay = atan2(accY, sqrt( pow(accX, 2) + pow(accZ, 2))) * 180 / M_PI;

// angles based on gyro (deg/s)
  
  gx = gx + gyrX / FREQ;
  gy = gy + gyrY / FREQ;
  gz = gz + gyrZ / FREQ;



  BTSerial.print(gyrX);
  BTSerial.print(", ");
  BTSerial.print(gyrY);
  BTSerial.print(", ");
  BTSerial.print(gyrZ);
  BTSerial.print(" || ");
  
  BTSerial.print(gx);
  BTSerial.print(", ");
  BTSerial.print(gy);
  BTSerial.print(", ");
  BTSerial.print(gz);
  BTSerial.write("     \r");

/*
  BTSerial.print(ax, 2);
  BTSerial.print(", ");
  BTSerial.println(ay, 2);
*/
  delay((1/FREQ) * 1000);
}


// --------------------------------------------------------
// MPU6050_read
//
// This is a common function to read multiple bytes 
// from an I2C device.
//
// It uses the boolean parameter for Wire.endTransMission()
// to be able to hold or release the I2C-bus. 
// This is implemented in Arduino 1.0.1.
//
// Only this function is used to read. 
// There is no function for a single byte.
//
int MPU6050_read(int start, uint8_t *buffer, int size)
{
  int i, n, error;

  Wire.beginTransmission(MPU6050_I2C_ADDRESS);
  n = Wire.write(start);
  if (n != 1)
    return (-10);

  n = Wire.endTransmission(false);    // hold the I2C-bus
  if (n != 0)
    return (n);

  // Third parameter is true: relase I2C-bus after data is read.
  Wire.requestFrom(MPU6050_I2C_ADDRESS, size, true);
  i = 0;
  while(Wire.available() && i<size)
  {
    buffer[i++]=Wire.read();
  }
  if ( i != size)
    return (-11);

  return (0);  // return : no error
}


// --------------------------------------------------------
// MPU6050_write
//
// This is a common function to write multiple bytes to an I2C device.
//
// If only a single register is written,
// use the function MPU_6050_write_reg().
//
// Parameters:
//   start : Start address, use a define for the register
//   pData : A pointer to the data to write.
//   size  : The number of bytes to write.
//
// If only a single register is written, a pointer
// to the data has to be used, and the size is
// a single byte:
//   int data = 0;        // the data to write
//   MPU6050_write (MPU6050_PWR_MGMT_1, &c, 1);
//
int MPU6050_write(int start, const uint8_t *pData, int size)
{
  int n, error;

  Wire.beginTransmission(MPU6050_I2C_ADDRESS);
  n = Wire.write(start);        // write the start address
  if (n != 1)
    return (-20);

  n = Wire.write(pData, size);  // write data bytes
  if (n != size)
    return (-21);

  error = Wire.endTransmission(true); // release the I2C-bus
  if (error != 0)
    return (error);

  return (0);         // return : no error
}

// --------------------------------------------------------
// MPU6050_write_reg
//
// An extra function to write a single register.
// It is just a wrapper around the MPU_6050_write()
// function, and it is only a convenient function
// to make it easier to write a single register.
//
int MPU6050_write_reg(int reg, uint8_t data)
{
  int error;

  error = MPU6050_write(reg, &data, 1);

  return (error);
}

