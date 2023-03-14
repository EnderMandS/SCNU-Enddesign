'''
Description: 
Autor: M
Date: 2023-03-13 21:54:04
LastEditors: M
LastEditTime: 2023-03-15 00:33:02
'''

import machine
from mpu import MPU6050

def main():
    machine.freq(240000000)
    print("\n\nThis is the program of MPU6050")
    print('CPU clock: %dHz' % machine.freq())

    # scl=18, sda=19
    i2c_obj = machine.I2C(0, freq=400000)
    mpu_obj = MPU6050(i2c=i2c_obj)
    print('MPU Init success')
    # print(mpu_obj.get_raw_array())

    # 偶校验 115200 tx=10, rx=9
    uart = machine.UART(1,baudrate=115200, txbuf=1024, rxbuf=129, \
                         parity=0 , timeout=100, timeout_char=100)
    print('UART Init success')
    
    while 1:
        data = mpu_obj.get_raw_values()
        print(uart.write(data))
    
if __name__ == '__main__':
    main()
    