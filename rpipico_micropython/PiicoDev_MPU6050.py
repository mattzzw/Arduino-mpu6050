# Class to read data from the Core Electronics PiicoDev Motion Sensor MPU-6050
# Ported to MicroPython by Peter Johnston and Michael Ruppeat Core Electronics APR 2021
# Original repo https://github.com/nickcoutsos/MPU-6050-Python

from PiicoDev_Unified import *
from math import sqrt, atan2

compat_str = '\nUnified PiicoDev library out of date.  Get the latest module: https://piico.dev/unified \n'

# Global Variables
_GRAVITIY_MS2 = 9.80665

# Scale Modifiers
_ACC_SCLR_2G = 16384.0
_ACC_SCLR_4G = 8192.0
_ACC_SCLR_8G = 4096.0
_ACC_SCLR_16G = 2048.0

_GYR_SCLR_250DEG = 131.0
_GYR_SCLR_500DEG = 65.5
_GYR_SCLR_1000DEG = 32.8
_GYR_SCLR_2000DEG = 16.4

# Pre-defined ranges
_ACC_RNG_2G = 0x00
_ACC_RNG_4G = 0x08
_ACC_RNG_8G = 0x10
_ACC_RNG_16G = 0x18

_GYR_RNG_250DEG = 0x00
_GYR_RNG_500DEG = 0x08
_GYR_RNG_1000DEG = 0x10
_GYR_RNG_2000DEG = 0x18

# MPU-6050 Registers
_PWR_MGMT_1 = 0x6B

_ACCEL_XOUT0 = 0x3B

_TEMP_OUT0 = 0x41

_GYRO_XOUT0 = 0x43

_ACCEL_CONFIG = 0x1C
_GYRO_CONFIG = 0x1B

_maxFails = 3

# Address
_MPU6050_ADDRESS = 0x68

def signedIntFromBytes(x, endian='big'):
    y = int.from_bytes(x, endian)
    if (y >= 0x8000):
        return -((65535 - y) + 1)
    else:
        return y
    

class PiicoDev_MPU6050(object):     
    def __init__(self, bus=None, freq=None, sda=None, scl=None, addr=_MPU6050_ADDRESS):
        self._failCount = 0
        self._terminatingFailCount = 0
        try:
            if compat_ind >= 1:
                pass
            else:
                print(compat_str)
        except:
            print(compat_str)
        self.i2c = create_unified_i2c(bus=bus, freq=freq, sda=sda, scl=scl)
        self.addr = addr
        try:
            # Wake up the MPU-6050 since it starts in sleep mode
            self.i2c.writeto_mem(self.addr, _PWR_MGMT_1, bytes([0x00]))
            sleep_ms(5)
        except Exception as e:
            print(i2c_err_str.format(self.addr))
            raise e
        self._accel_range = self.get_accel_range(True)
        self._gyro_range = self.get_gyro_range(True)

    def _readData(self, register):
        failCount = 0
        while failCount < _maxFails:
            try:
                sleep_ms(10)
                data = self.i2c.readfrom_mem(self.addr, register, 6)
                break
            except:
                failCount = failCount + 1
                self._failCount = self._failCount + 1
                if failCount >= _maxFails:
                    self._terminatingFailCount = self._terminatingFailCount + 1
                    print(i2c_err_str.format(self.addr))
                    return {'x': float('NaN'), 'y': float('NaN'), 'z': float('NaN')} 
        x = signedIntFromBytes(data[0:2])
        y = signedIntFromBytes(data[2:4])
        z = signedIntFromBytes(data[4:6])
        return {'x': x, 'y': y, 'z': z}

    # Reads the temperature from the onboard temperature sensor of the MPU-6050.
    # Returns the temperature [degC].
    def read_temperature(self):
        try:
            rawData = self.i2c.readfrom_mem(self.addr, _TEMP_OUT0, 2)
            raw_temp = (signedIntFromBytes(rawData, 'big'))
        except:
            print(i2c_err_str.format(self.addr))
            return float('NaN')
        actual_temp = (raw_temp / 340) + 36.53
        return actual_temp

    # Sets the range of the accelerometer
    # accel_range : the range to set the accelerometer to. Using a pre-defined range is advised.
    def set_accel_range(self, accel_range):
        self.i2c.writeto_mem(self.addr, _ACCEL_CONFIG, bytes([accel_range]))
        self._accel_range = accel_range

    # Gets the range the accelerometer is set to.
    # raw=True: Returns raw value from the ACCEL_CONFIG register
    # raw=False: Return integer: -1, 2, 4, 8 or 16. When it returns -1 something went wrong.
    def get_accel_range(self, raw = False):
        # Get the raw value
        raw_data = self.i2c.readfrom_mem(self.addr, _ACCEL_CONFIG, 2)
        
        if raw is True:
            return raw_data[0]
        elif raw is False:
            if raw_data[0] == _ACC_RNG_2G:
                return 2
            elif raw_data[0] == _ACC_RNG_4G:
                return 4
            elif raw_data[0] == _ACC_RNG_8G:
                return 8
            elif raw_data[0] == _ACC_RNG_16G:
                return 16
            else:
                return -1

    # Reads and returns the X, Y and Z values from the accelerometer.
    # Returns dictionary data in g or m/s^2 (g=False)
    def read_accel_data(self, g = False):         
        accel_data = self._readData(_ACCEL_XOUT0)
        accel_range = self._accel_range
        scaler = None
        if accel_range == _ACC_RNG_2G:
            scaler = _ACC_SCLR_2G
        elif accel_range == _ACC_RNG_4G:
            scaler = _ACC_SCLR_4G
        elif accel_range == _ACC_RNG_8G:
            scaler = _ACC_SCLR_8G
        elif accel_range == _ACC_RNG_16G:
            scaler = _ACC_SCLR_16G
        else:
            print("Unkown range - scaler set to _ACC_SCLR_2G")
            scaler = _ACC_SCLR_2G

        x = accel_data['x'] / scaler
        y = accel_data['y'] / scaler
        z = accel_data['z'] / scaler

        if g is True:
            return {'x': x, 'y': y, 'z': z}
        elif g is False:
            x = x * _GRAVITIY_MS2
            y = y * _GRAVITIY_MS2
            z = z * _GRAVITIY_MS2
            return {'x': x, 'y': y, 'z': z}

    def read_accel_abs(self, g=False):
        d=self.read_accel_data(g)
        return sqrt(d['x']**2+d['y']**2+d['z']**2)

    def set_gyro_range(self, gyro_range):
        self.i2c.writeto_mem(self.addr, _GYRO_CONFIG, bytes([gyro_range]))
        self._gyro_range = gyro_range

    # Gets the range the gyroscope is set to.
    # raw=True: return raw value from GYRO_CONFIG register
    # raw=False: return range in deg/s
    def get_gyro_range(self, raw = False):
        # Get the raw value
        raw_data = self.i2c.readfrom_mem(self.addr, _GYRO_CONFIG, 2)

        if raw is True:
            return raw_data[0]
        elif raw is False:
            if raw_data[0] == _GYR_RNG_250DEG:
                return 250
            elif raw_data[0] == _GYR_RNG_500DEG:
                return 500
            elif raw_data[0] == _GYR_RNG_1000DEG:
                return 1000
            elif raw_data[0] == _GYR_RNG_2000DEG:
                return 2000
            else:
                return -1

    # Gets and returns the X, Y and Z values from the gyroscope.
    # Returns the read values in a dictionary.
    def read_gyro_data(self):
        gyro_data = self._readData(_GYRO_XOUT0)
        gyro_range = self._gyro_range
        scaler = None
        if gyro_range == _GYR_RNG_250DEG:
            scaler = _GYR_SCLR_250DEG
        elif gyro_range == _GYR_RNG_500DEG:
            scaler = _GYR_SCLR_500DEG
        elif gyro_range == _GYR_RNG_1000DEG:
            scaler = _GYR_SCLR_1000DEG
        elif gyro_range == _GYR_RNG_2000DEG:
            scaler = _GYR_SCLR_2000DEG
        else:
            print("Unkown range - scaler set to _GYR_SCLR_250DEG")
            scaler = _GYR_SCLR_250DEG

        x = gyro_data['x'] / scaler
        y = gyro_data['y'] / scaler
        z = gyro_data['z'] / scaler

        return {'x': x, 'y': y, 'z': z}

    def read_angle(self): # returns radians. orientation matches silkscreen
        a=self.read_accel_data()
        x=atan2(a['y'],a['z'])
        y=atan2(-a['x'],a['z'])
        return {'x': x, 'y': y}