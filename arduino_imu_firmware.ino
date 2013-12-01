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

#define FREQ  20.0 // sample freq in Hz

// Bluetooth transmitter 
SoftwareSerial BTSerial(2, 3); // RX | TX

// global angle, gyro derived
double gSensitivity = 65.5; // for 500 deg/s, check data sheet
double gx = 0, gy = 0, gz = 0;
double gyrX = 0, gyrY = 0, gyrZ = 0;
int16_t accX = 0, accY = 0, accZ = 0;

double gyrXoffs = -281.00, gyrYoffs = 18.00, gyrZoffs = -83.00;

void setup()
{      
  int error;
  uint8_t c;
  uint8_t sample_div;

  BTSerial.begin(38400);
  Serial.begin(38400);

  pinMode(13, OUTPUT); // debug led

  // Initialize the 'Wire' class for the I2C-bus.
  Wire.begin();

  // PWR_MGMT_1:
  // wake up 
  MPU6050_write_reg (0x6b, 0x00);

  // CONFIG:
  // Low pass filter samples, 1khz sample rate
  MPU6050_write_reg (0x1a, 0x01);

  // GYRO_CONFIG:
  // 500 deg/s, FS_SEL=1
  // This means 65.5 LSBs/deg/s
  MPU6050_write_reg(0x1b, 0x08);

  // CONFIG:
  // set sample rate
  // sample rate FREQ = Gyro sample rate / (sample_div + 1)
  // 1kHz / (div + 1) = FREQ  
  // reg_value = 1khz/FREQ - 1
  sample_div = 1000 / FREQ - 1;
  MPU6050_write_reg (0x19, sample_div);

  digitalWrite(13, HIGH);
  calibrate();
  digitalWrite(13, LOW);
}

void loop()
{
  int error;
  double dT;
  double ax, ay, az;
  unsigned long start_time, end_time;

  start_time = millis();

  read_sensor_data();

  // angles based on accelerometer
  ay = atan2(accX, sqrt( pow(accY, 2) + pow(accZ, 2))) * 180 / M_PI;
  ax = atan2(accY, sqrt( pow(accX, 2) + pow(accZ, 2))) * 180 / M_PI;

  // angles based on gyro (deg/s)

  gx = gx + gyrX / FREQ;
  gy = gy - gyrY / FREQ;
  gz = gz + gyrZ / FREQ;


  // complementary filter
  gx = gx * 0.96 + ax * 0.04;
  gy = gy * 0.96 + ay * 0.04;



#if 0
  Serial.print(ax, 2);
  Serial.print(", ");
  Serial.print(ay, 2);
  Serial.print(" || ");
  Serial.print(gx, 2);
  Serial.print(", ");
  Serial.print(gy, 2);
  Serial.print(", ");
  Serial.print(gz, 2);
  Serial.write("     \r");
#endif


  // check if there is some kind of request 
  // from the other side...
  if(BTSerial.available())
  {
    char rx_char;
    // dummy read
    rx_char = BTSerial.read();
    // we have to send data, as requested
    if (rx_char == '.'){
      digitalWrite(13, HIGH);
      BTSerial.print(gx, 2);
      BTSerial.print(", ");
      BTSerial.print(gy, 2);
      BTSerial.print(", ");
      BTSerial.println(gz, 2);
      digitalWrite(13, LOW);
    }
    // reset z gyro axis
    if (rx_char == 'z'){
      gz = 0;
    }  
  }

  end_time = millis();

  // remaining time to complete sample time
  delay(((1/FREQ) * 1000) - (end_time - start_time));
}


void calibrate(){

  int x;
  long xSum = 0, ySum = 0, zSum = 0;
  uint8_t i2cData[6]; 
  int num = 500;
  uint8_t error;

  for (x = 0; x < num; x++){

    error = MPU6050_read(0x43, i2cData, 6);
    if(error!=0)
      return;

    xSum += ((i2cData[0] << 8) | i2cData[1]);
    ySum += ((i2cData[2] << 8) | i2cData[3]);
    zSum += ((i2cData[4] << 8) | i2cData[5]);
  }
  gyrXoffs = xSum / num;
  gyrYoffs = ySum / num;
  gyrZoffs = zSum / num;

#if 0
  BTSerial.println("Calibration result:");
  BTSerial.print(gyrXoffs);
  BTSerial.print(", ");
  BTSerial.print(gyrYoffs);
  BTSerial.print(", ");
  BTSerial.println(gyrZoffs);
 
  BTSerial.print(accXoffs);
  BTSerial.print(", ");
  BTSerial.print(accYoffs);
  BTSerial.print(", ");
  BTSerial.println(accZoffs);
  #endif

} 

void read_sensor_data(){
 uint8_t i2cData[14];
 uint8_t error;
  // read imu data
  error = MPU6050_read(0x3b, i2cData, 14);
  if(error!=0)
    return;

  // assemble 16 bit sensor data
  accX = ((i2cData[0] << 8) | i2cData[1]);
  accY = ((i2cData[2] << 8) | i2cData[3]);
  accZ = ((i2cData[4] << 8) | i2cData[5]);

  gyrX = (((i2cData[8] << 8) | i2cData[9]) - gyrXoffs) / gSensitivity;
  gyrY = (((i2cData[10] << 8) | i2cData[11]) - gyrYoffs) / gSensitivity;
  gyrZ = (((i2cData[12] << 8) | i2cData[13]) - gyrZoffs) / gSensitivity;
 
}


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


int MPU6050_write_reg(int reg, uint8_t data)
{
  int error;

  error = MPU6050_write(reg, &data, 1);

  return (error);
}

