import math
from PiicoDev_MPU6050 import PiicoDev_MPU6050
import time
import sys
import uselect

# Initialize MPU6050 (bus=1, sda=14, scl=15)
mpu = PiicoDev_MPU6050(bus=1, sda=14, scl=15)

# Calibrate gyro to calculate offsets in deg/s
def calibrate_gyro(num_samples=500):
    gx_total = gy_total = gz_total = 0.0
    for _ in range(num_samples):
        gyro = mpu.read_gyro_data()
        gx_total += gyro['x']
        gy_total += gyro['y']
        gz_total += gyro['z']
        time.sleep_ms(1)
    return (gx_total/num_samples, gy_total/num_samples, gz_total/num_samples)

gyrX_offset, gyrY_offset, gyrZ_offset = calibrate_gyro()

# Complementary filter variables
gx = gy = gz = 0.0
FREQ = 30.0
dt = 1.0 / FREQ

# Setup non-blocking input for serial requests
poll = uselect.poll()
poll.register(sys.stdin, uselect.POLLIN)

while True:
    start_time = time.ticks_ms()
    
    # Read accelerometer and calculate angles
    accel = mpu.read_accel_data(g=True)
    ax, ay, az = accel['x'], accel['y'], accel['z']
    
    roll_acc = math.degrees(math.atan2(ay, math.sqrt(ax**2 + az**2)))
    pitch_acc = math.degrees(math.atan2(ax, math.sqrt(ay**2 + az**2)))
    
    # Read and calibrate gyro data (deg/s)
    gyro = mpu.read_gyro_data()
    gyrX = gyro['x'] - gyrX_offset
    gyrY = -(gyro['y'] - gyrY_offset)  # Negative sign to match Arduino orientation
    gyrZ = gyro['z'] - gyrZ_offset
    
    # Integrate gyro data
    gx += gyrX * dt
    gy += gyrY * dt
    gz += gyrZ * dt
    
    # Apply complementary filter (96% gyro, 4% accelerometer)
    alpha = 0.96
    gx = gx * alpha + roll_acc * (1 - alpha)
    gy = gy * alpha + pitch_acc * (1 - alpha)
    
    # Check for incoming serial commands
    if poll.poll(0):
        cmd = sys.stdin.read(1)
        if cmd == '.':
            print("{:.2f}, {:.2f}, {:.2f}".format(gx, gy, gz))
        elif cmd == 'z':
            gz = 0  # Reset yaw
    
    # Maintain sample rate
    elapsed = time.ticks_diff(time.ticks_ms(), start_time)
    remaining = int(1000/FREQ) - elapsed
    if remaining > 0:
        time.sleep_ms(remaining)