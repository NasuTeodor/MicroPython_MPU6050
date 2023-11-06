# imu.py MicroPython driver for the InvenSense inertial measurement units
# This is the base class
# Adapted from Sebastian Plamauer's MPU9150 driver:
# https://github.com/micropython-IMU/micropython-mpu9150.git
# Authors Peter Hinch, Sebastian Plamauer
# V0.2 17th May 2017 Platform independent: utime and machine replace pyb

'''
mpu9250 is a micropython module for the InvenSense MPU9250 sensor.
It measures acceleration, turn rate and the magnetic field in three axis.
mpu9150 driver modified for the MPU9250 by Peter Hinch

The MIT License (MIT)
Copyright (c) 2014 Sebastian Plamauer, oeplse@gmail.com, Peter Hinch
Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:
The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.
THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
'''

# User access is now by properties e.g.
# myimu = MPU9250('X')
# magx = myimu.mag.x
# accelxyz = myimu.accel.xyz
# Error handling: on code used for initialisation, abort with message
# At runtime try to continue returning last good data value. We don't want aircraft
# crashing. However if the I2C has crashed we're probably stuffed.

from utime import sleep_ms, ticks_ms, ticks_diff
from math import sqrt, atan2
from machine import I2C


class MPUException(OSError):
    '''
    Exception for MPU devices
    '''
    pass

def bytes_toint(msb, lsb):
    '''
    Convert two bytes to signed integer (big endian)
    for little endian reverse msb, lsb arguments
    Can be used in an interrupt handler
    '''
    if not msb & 0x80:
        return msb << 8 | lsb  # +ve
    return - (((msb ^ 255) << 8) | (lsb ^ 255) + 1)

def wrap(angle):
    while (angle > +180):
        angle -= 360
    while (angle < -180):
        angle += 360
    return angle

def angle_average(wa, a, wb, b):
	return wrap(wa * a + wb * (a + wrap(b-a)))

class MPU6050(object):
    '''
    Module for InvenSense IMUs. Base class implements MPU6050 6DOF sensor, with
    features common to MPU9150 and MPU9250 9DOF sensors.
    '''
    __CALIBRATION_MEASURES = 500

    __DEFAULT_ACCEL_COEFF = 0.02
    __DEFAULT_GYRO_COEFF = 0.98

    __ACCEL_TRANSFORMATION_NUMBER = 0.00006103515
    # __GYRO_TRANSFORMATION_NUMBER = 0.01525878906
    
    # UPDATED TRANSFORMATION NUMBER FOR THE SENSIVITY
    # __GYRO_TRANSFORMATION_NUMBER = 0.00855878906
    __GYRO_TRANSFORMATION_NUMBER = 0.00852878906

    __RAD_TO_DEG = 57.2957795131
    
    _rawAccX = 0
    _rawAccY = 0
    _rawAccZ = 0

    _rawGyroX = 0
    _rawGyroY = 0
    _rawGyroZ = 0
    
    _accX = 0
    _accY = 0
    _accZ = 0

    _angGyroX = 0
    _angGyroY = 0
    _angGyroZ = 0

    _angX = 0
    _angY = 0
    _angZ = 0

    __dt = 0
    __intervalStart = 0

    __gyroXOffset = 0
    __gyroYOffset = 0
    __gyroZOffset = 0

    __filterAccelCoeff = 0
    __filterGyroCoeff = 0

    _I2Cerror = "I2C failure when communicating with IMU"
    _mpu_addr = (104, 105)  # addresses of MPU9150/MPU6050. There can be two devices
    _chip_id = 104

    def __init__(self, side_str, device_addr=None):

        self.buf1 = bytearray(1)                # Pre-allocated buffers for reads: allows reads to
        self.buf2 = bytearray(2)                # be done in interrupt handlers
        self.buf3 = bytearray(3)
        self.buf6 = bytearray(6)

        sleep_ms(200)                           # Ensure PSU and device have settled
        if isinstance(side_str, str):           # Non-pyb targets may use other than X or Y
            self._mpu_i2c = I2C(side_str)
        elif hasattr(side_str, 'readfrom'):     # Soft or hard I2C instance. See issue #3097
            self._mpu_i2c = side_str
        else:
            raise ValueError("Invalid I2C instance")

        if device_addr is None:
            devices = set(self._mpu_i2c.scan())
            mpus = devices.intersection(set(self._mpu_addr))
            number_of_mpus = len(mpus)
            if number_of_mpus == 0:
                raise MPUException("No MPU's detected")
            elif number_of_mpus == 1:
                self.mpu_addr = mpus.pop()
            else:
                raise ValueError("Two MPU's detected: must specify a device address")
        else:
            if device_addr not in (0, 1):
                raise ValueError('Device address must be 0 or 1')
            self.mpu_addr = self._mpu_addr[device_addr]

        self.chip_id                     # Test communication by reading chip_id: throws exception on error
        # Can communicate with chip. Set it up.

    def Initialize(self):
        '''
        Start the MPU6050 and read data for the first time
        '''
        self.__filterAccelCoeff = self.__DEFAULT_ACCEL_COEFF
        self.__filterGyroCoeff = self.__DEFAULT_GYRO_COEFF

        self.accel_range = 0                    # default to highest sensitivity
        self.gyro_range = 0                     # Likewise for gyro

        self.wake()                             # wake it up

        self.__intervalStart = ticks_ms()

    def Calibrate(self):
        '''
        Do multiple reads and calculate the average error to be used
        for later calculations.
        '''
        sumGyroX = 0
        sumGyroY = 0
        sumGyroZ = 0

        i = 0
        while( i < self.__CALIBRATION_MEASURES):
            self.update_accel()
            self.update_gyro()
            sumGyroX += self._rawGyroX
            sumGyroY += self._rawGyroY
            sumGyroZ += self._rawGyroZ
            i+=1
            sleep_ms(1)
        
        sumGyroX /= self.__CALIBRATION_MEASURES
        sumGyroY /= self.__CALIBRATION_MEASURES
        sumGyroZ /= self.__CALIBRATION_MEASURES

        self.__gyroXOffset = sumGyroX
        self.__gyroYOffset = sumGyroY
        self.__gyroZOffset = sumGyroZ

    def read(self):
        '''
        Update the acceleration and gyroscope data of the MPU6050.\n
        TO AVOID UNREGISTERED MOVEMENT IT HAS TO BE CALLED CONTINUOUS.
        '''
        self.update_accel()
        self.update_gyro()

        accX = self._rawAccX * self.__ACCEL_TRANSFORMATION_NUMBER
        accY = self._rawAccY * self.__ACCEL_TRANSFORMATION_NUMBER
        accZ = self._rawAccZ * self.__ACCEL_TRANSFORMATION_NUMBER

        gyroX = (self._rawGyroX - self.__gyroXOffset) * self.__GYRO_TRANSFORMATION_NUMBER
        gyroY = (self._rawGyroY - self.__gyroYOffset) * self.__GYRO_TRANSFORMATION_NUMBER
        gyroZ = (self._rawGyroZ - self.__gyroZOffset) * self.__GYRO_TRANSFORMATION_NUMBER

        self._angAccX = wrap((atan2(accY, sqrt(accZ * accZ + accX * accX))) * self.__RAD_TO_DEG)
        self._angAccY = wrap((-atan2(accX, sqrt(accZ * accZ + accY * accY))) * self.__RAD_TO_DEG)
        
        self.__dt = (ticks_diff(ticks_ms(), self.__intervalStart)) * 0.001
        self._angGyroX = wrap(self._angGyroX + gyroX * self.__dt)
        self._angGyroY = wrap(self._angGyroY + gyroY * self.__dt)
        self._angGyroZ = wrap(self._angGyroZ + gyroZ * self.__dt)

        self._angX = angle_average(self.__filterAccelCoeff, self._angAccX, self.__filterGyroCoeff, self._angX + gyroX * self.__dt)
        self._angY = angle_average(self.__filterAccelCoeff, self._angAccY, self.__filterGyroCoeff, self._angY + gyroY * self.__dt)
        self._angZ = self._angGyroZ

        self.__intervalStart = ticks_ms()

    # read from device
    def _read(self, buf, memaddr, addr):        # addr = I2C device address, memaddr = memory location within the I2C device
        '''
        Read bytes to pre-allocated buffer Caller traps OSError.
        '''
        self._mpu_i2c.readfrom_mem_into(addr, memaddr, buf)

    # write to device
    def _write(self, data, memaddr, addr):
        '''
        Perform a memory write. Caller should trap OSError.
        '''
        self.buf1[0] = data
        self._mpu_i2c.writeto_mem(addr, memaddr, self.buf1)

    # wake
    def wake(self):
        '''
        Wakes the device.
        '''
        try:
            self._write(0x01, 0x6B, self.mpu_addr)  # Use best clock source
        except OSError:
            raise MPUException(self._I2Cerror)
        return 'awake'

    # mode
    def sleep(self):
        '''
        Sets the device to sleep mode.
        '''
        try:
            self._write(0x40, 0x6B, self.mpu_addr)
        except OSError:
            raise MPUException(self._I2Cerror)
        return 'asleep'

    # chip_id
    @property
    def chip_id(self):
        '''
        Returns Chip ID
        '''
        try:
            self._read(self.buf1, 0x75, self.mpu_addr)
        except OSError:
            raise MPUException(self._I2Cerror)
        chip_id = int(self.buf1[0])
        if chip_id != self._chip_id:
            raise ValueError('Bad chip ID retrieved: MPU communication failure')
        return chip_id

    # accelerometer range
    @property
    def accel_range(self):
        '''
        Accelerometer range
        Value:              0   1   2   3
        for range +/-:      2   4   8   16  g
        '''
        try:
            self._read(self.buf1, 0x1C, self.mpu_addr)
            ari = self.buf1[0]//8
        except OSError:
            raise MPUException(self._I2Cerror)
        return ari

    @accel_range.setter
    def accel_range(self, accel_range):
        '''
        Set accelerometer range
        Pass:               0   1   2   3
        for range +/-:      2   4   8   16  g
        '''
        ar_bytes = (0x00, 0x08, 0x10, 0x18)
        if accel_range in range(len(ar_bytes)):
            try:
                self._write(ar_bytes[accel_range], 0x1C, self.mpu_addr)
            except OSError:
                raise MPUException(self._I2Cerror)
        else:
            raise ValueError('accel_range can only be 0, 1, 2 or 3')

    # gyroscope range
    @property
    def gyro_range(self):
        '''
        Gyroscope range
        Value:              0   1   2    3
        for range +/-:      250 500 1000 2000  degrees/second
        '''
        # set range
        try:
            self._read(self.buf1, 0x1B, self.mpu_addr)
            gri = self.buf1[0]//8
        except OSError:
            raise MPUException(self._I2Cerror)
        return gri

    @gyro_range.setter
    def gyro_range(self, gyro_range):
        '''
        Set gyroscope range
        Pass:               0   1   2    3
        for range +/-:      250 500 1000 2000  degrees/second
        '''
        gr_bytes = (0x00, 0x08, 0x10, 0x18)
        if gyro_range in range(len(gr_bytes)):
            try:
                self._write(gr_bytes[gyro_range], 0x1B, self.mpu_addr)  # Sets fchoice = b11 which enables filter
            except OSError:
                raise MPUException(self._I2Cerror)
        else:
            raise ValueError('gyro_range can only be 0, 1, 2 or 3')

    def update_accel(self):
        '''
        For use in interrupt handlers. Sets to signed
        unscaled integer accelerometer values
        '''
        self._read(self.buf6, 0x3B, self.mpu_addr)
        self._rawAccX = bytes_toint(self.buf6[0], self.buf6[1])
        self._rawAccY = bytes_toint(self.buf6[2], self.buf6[3])
        self._rawAccZ = bytes_toint(self.buf6[4], self.buf6[5])

    def update_gyro(self):
        '''
        For use in interrupt handlers. Sets to signed
        unscaled integer gyro values. Error trapping disallowed.
        '''
        self._read(self.buf6, 0x43, self.mpu_addr)
        self._rawGyroX = bytes_toint(self.buf6[0], self.buf6[1])
        self._rawGyroY = bytes_toint(self.buf6[2], self.buf6[3])
        self._rawGyroZ = bytes_toint(self.buf6[4], self.buf6[5])
