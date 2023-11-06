from mpu6050 import MPU6050
from machine import Pin, I2C

i2c = I2C(id=1, sda=Pin(26), scl=Pin(27), freq=40000)
mpu = MPU6050(i2c)

print("Initializing")
mpu.Initialize()
print("Calibrating...")
mpu.Calibrate()
print("Done calibrating.")

while True:
    mpu.read()
    
    gx = mpu._angX
    gy = mpu._angY
    gz = mpu._angZ

    print(gz)
