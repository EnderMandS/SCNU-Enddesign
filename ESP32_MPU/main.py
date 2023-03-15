'''
Description: 
Autor: M
Date: 2023-03-13 21:54:04
LastEditors: M
LastEditTime: 2023-03-15 22:32:58
'''

import machine, time
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

    # 偶校验 tx17 rx16
    uart = machine.UART(2,baudrate=115200, txbuf=1024, rxbuf=129, parity=0)
    print('UART Init success')

    # GPIO Init
    # p13 = machine.Pin(13, machine.Pin.IN)   # 用来退出while 1
    
    while 1:
        mpu_data = mpu_obj.get_raw_values() #14
        t = time.ticks_ms().to_bytes(4, 'little') #4
        s = (sum(mpu_data + t)%256).to_bytes(1, 'little') #1
        data = mpu_data + t + s + b'END' # 14+4+1+3=22
        uart.write(data)
    
if __name__ == '__main__':
    main()
    