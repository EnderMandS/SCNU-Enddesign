'''
Description: 
Autor: M
Date: 2023-03-13 21:54:04
LastEditors: M
LastEditTime: 2023-03-14 19:32:27
'''

import machine
from systim import SYSTIM
from mpu import MPU6050

def main():
    machine.freq(240000000)
    print("\n\nThis is the program of MPU6050")
    print('CPU clock: %dHz\n' % machine.freq())
    # tim = SYSTIM()
    # tim.go_on()
    i2c_obj = machine.I2C(0, freq=400000)
    # print(i2c_obj.scan())
    mpu_obj = MPU6050(i2c=i2c_obj)
    print(mpu_obj.get_values())
    
if __name__ == '__main__':
    main()
    