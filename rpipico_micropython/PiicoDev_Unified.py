'''
PiicoDev.py: Unifies I2C drivers for different builds of MicroPython
Changelog:
    - 2021       M.Ruppe - Initial Unified Driver
    - 2022-10-13 P.Johnston - Add helptext to run i2csetup script on Raspberry Pi 
    - 2022-10-14 M.Ruppe - Explicitly set default I2C initialisation parameters for machine-class (Raspberry Pi Pico + W)
    - 2023-01-31 L.Howell - Add minimal support for ESP32
    - 2023-05-17 M.Ruppe - Make I2CUnifiedMachine() more flexible on initialisation. Frequency is optional.
    - 2023-12-20 M.Taylor - added scan() function for quick userland test of connected i2c modules
'''
import os
_SYSNAME = os.uname().sysname
compat_ind = 1
i2c_err_str = 'PiicoDev could not communicate with module at address 0x{:02X}, check wiring'
setupi2c_str = ', run "sudo curl -L https://piico.dev/i2csetup | bash". Suppress this warning by setting suppress_warnings=True'

if _SYSNAME == 'microbit':
    from microbit import i2c
    from utime import sleep_ms
    
elif _SYSNAME == 'Linux':
    from smbus2 import SMBus, i2c_msg
    from time import sleep
    from math import ceil
    
    def sleep_ms(t):
        sleep(t/1000)

else:
    from machine import I2C, Pin
    from utime import sleep_ms

class I2CBase:
    def writeto_mem(self, addr, memaddr, buf, *, addrsize=8):
        raise NotImplementedError('writeto_mem')

    def readfrom_mem(self, addr, memaddr, nbytes, *, addrsize=8):
        raise NotImplementedError('readfrom_mem')

    def write8(self, addr, buf, stop=True):
        raise NotImplementedError('write')

    def read16(self, addr, nbytes, stop=True):
        raise NotImplementedError('read')

    def __init__(self, bus=None, freq=None, sda=None, scl=None):
        raise NotImplementedError('__init__')

class I2CUnifiedMachine(I2CBase):
    def __init__(self, bus=None, freq=None, sda=None, scl=None):
        if _SYSNAME == 'esp32' and (bus is None or sda is None or scl is None):
            raise Exception('Please input bus, machine.pin SDA, and SCL objects to use ESP32')
        
        if freq is None: freq = 400_000
        if not isinstance(freq, (int)):
            raise ValueError("freq must be an Int")
        if freq < 400_000: print("\033[91mWarning: minimum freq 400kHz is recommended if using OLED module.\033[0m")
        if bus is not None and sda is not None and scl is not None:
            print('Using supplied bus, sda, and scl to create machine.I2C() with freq: {} Hz'.format(freq))
            self.i2c = I2C(bus, freq=freq, sda=sda, scl=scl)
        elif bus is None and sda is None and scl is None:
            self.i2c = I2C(0, scl=Pin(9), sda=Pin(8), freq=freq) # RPi Pico in Expansion Board
        else:
            raise Exception("Please provide at least bus, sda, and scl")

        self.writeto_mem = self.i2c.writeto_mem
        self.readfrom_mem = self.i2c.readfrom_mem

    def write8(self, addr, reg, data):
        if reg is None:
            self.i2c.writeto(addr, data)
        else:
            self.i2c.writeto(addr, reg + data)
            
    def read16(self, addr, reg):
        self.i2c.writeto(addr, reg, False)
        return self.i2c.readfrom(addr, 2)
        
    def scan(self):
        print([hex(i) for i in self.i2c.scan()])

class I2CUnifiedMicroBit(I2CBase):
    def __init__(self, freq=None):
        if freq is not None:
            print('Initialising I2C freq to {}'.format(freq))
            microbit.i2c.init(freq=freq)
            
    def writeto_mem(self, addr, memaddr, buf, *, addrsize=8):
        ad = memaddr.to_bytes(addrsize // 8, 'big')  # pad address for eg. 16 bit
        i2c.write(addr, ad + buf)
        
    def readfrom_mem(self, addr, memaddr, nbytes, *, addrsize=8):
        ad = memaddr.to_bytes(addrsize // 8, 'big')  # pad address for eg. 16 bit
        i2c.write(addr, ad, repeat=True)
        return i2c.read(addr, nbytes)    
    
    def write8(self, addr, reg, data):
        if reg is None:
            i2c.write(addr, data)
        else:
            i2c.write(addr, reg + data)

    def read16(self, addr, reg):
        i2c.write(addr, reg, repeat=True)
        return i2c.read(addr, 2)
            
    def scan(self):
        print([hex(i) for i in self.i2c.scan()])

class I2CUnifiedLinux(I2CBase):
    def __init__(self, bus=None, suppress_warnings=True):
        if suppress_warnings == False:
            with open('/boot/config.txt') as config_file:
                if 'dtparam=i2c_arm=on' in config_file.read():
                    pass
                else:
                    print('I2C is not enabled. To enable' + setupi2c_str)
                config_file.close()
            with open('/boot/config.txt') as config_file:
                if 'dtparam=i2c_arm_baudrate=400000' in config_file.read():
                    pass
                else:
                    print('Slow baudrate detected. If glitching occurs' + setupi2c_str)
                config_file.close()
        if bus is None:
            bus = 1
        self.i2c = SMBus(bus)

    def readfrom_mem(self, addr, memaddr, nbytes, *, addrsize=8):
        data = [None] * nbytes # initialise empty list
        self.smbus_i2c_read(addr, memaddr, data, nbytes, addrsize=addrsize)
        return data
    
    def writeto_mem(self, addr, memaddr, buf, *, addrsize=8):
        self.smbus_i2c_write(addr, memaddr, buf, len(buf), addrsize=addrsize)
    
    def smbus_i2c_write(self, address, reg, data_p, length, addrsize=8):
        ret_val = 0
        data = []
        for index in range(length):
            data.append(data_p[index])
        if addrsize == 8:
            msg_w = i2c_msg.write(address, [reg] + data)
        elif addrsize == 16:
            msg_w = i2c_msg.write(address, [reg >> 8, reg & 0xff] + data)
        else:
            raise Exception('address must be 8 or 16 bits long only')
        self.i2c.i2c_rdwr(msg_w)
        return ret_val
        
    def smbus_i2c_read(self, address, reg, data_p, length, addrsize=8):
        ret_val = 0
        if addrsize == 8:
            msg_w = i2c_msg.write(address, [reg]) # warning this is set up for 16-bit addresses
        elif addrsize == 16:
            msg_w = i2c_msg.write(address, [reg >> 8, reg & 0xff]) # warning this is set up for 16-bit addresses
        else:
            raise Exception('address must be 8 or 16 bits long only')
        msg_r = i2c_msg.read(address, length)
        self.i2c.i2c_rdwr(msg_w, msg_r)
        if ret_val == 0:
            for index in range(length):
                data_p[index] = ord(msg_r.buf[index])
        return ret_val
    
    def write8(self, addr, reg, data):
        if reg is None:
            d = int.from_bytes(data, 'big')
            self.i2c.write_byte(addr, d)
        else:
            r = int.from_bytes(reg, 'big')
            d = int.from_bytes(data, 'big')
            self.i2c.write_byte_data(addr, r, d)
    
    def read16(self, addr, reg):
        regInt = int.from_bytes(reg, 'big')
        return self.i2c.read_word_data(addr, regInt).to_bytes(2, byteorder='little', signed=False)

    def scan(self):
        print([hex(i) for i in self.i2c.scan()])


def create_unified_i2c(bus=None, freq=None, sda=None, scl=None, suppress_warnings=True):
    if _SYSNAME == 'microbit':
        i2c = I2CUnifiedMicroBit(freq=freq)
    elif _SYSNAME == 'Linux':
        i2c = I2CUnifiedLinux(bus=bus, suppress_warnings=suppress_warnings)
    else:
        i2c = I2CUnifiedMachine(bus=bus, freq=freq, sda=sda, scl=scl)
    return i2c