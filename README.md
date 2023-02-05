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

You need:
* MPU6050 6DOF Sensor Module "GY-521" for 3,89EUR (ebay, sent from China)
Nice one. LDO regulator on board, takes 5V.
* Data Sheet / Register Map http://invensense.com/mems/gyro/documents/RM-MPU-6000A.pdf
* ArduinoBoard - Gotta love 'em

This was one of my first experiments. I only used the accelerometer. The reaction is very jerky and not precise at all.

[![Raw accelerotmeter data](https://img.youtube.com/vi/hf3emXvTad8/0.jpg)](https://www.youtube.com/watch?v=hf3emXvTad8)

Additionally using the gyros and mixing accelerometer and gyro data with a complementary filter improved precision, smoothness and response sensitivity a lot. As an option I also added yaw data from the gyro but that data is not filtered hence the error accumulates up pretty quickly.


[![Filtered accelerotmeter and gyro data](https://img.youtube.com/vi/yqFfmwVufMo/0.jpg)](https://www.youtube.com/watch?v=yqFfmwVufMo)

This is about 10 years old stuff - Here's the wayback link to my discontinued wiki:
http://web.archive.org/web/20190626080855/http://mattzz.no-ip.org/wiki/Projects/PlayingWithInertialMeasurementUnits
