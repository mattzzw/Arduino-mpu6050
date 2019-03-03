# Arduino with IMU 
Playing around with a 6DOF IMU (MPU-6050), Arduino, Python and OpenGL

The Arduino Uno is sending pitch and roll data via bluetooth. A python script is receiving the data and displaying a little cube accordingly. The IMU data consists of gyro and accelerometer data, processed by a complementary filter.

## Prerequisite Python Libraries
1. PyOpenGL 
2. Pygame
3. pySerial

## Deployment 
1. connect Arduion with MPU-6050 (connection diagram can be found at https://bit.ly/2VqX6p5)
2. Connect the Arduino MPU-6050 bundle to PC
2. In Arduino IDE, uploading **arduino_imu_firmware.ino**
3. In any Python IDE, run **boxctrl_6d0f_imu.py**

## Reference
For more information please visit http://mattzz.no-ip.org/wiki/Projects/PlayingWithInertialMeasurementUnits
