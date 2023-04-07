'''
Description: 
Autor: M
Date: 2023-03-13 21:55:43
LastEditors: M
LastEditTime: 2023-04-07 15:50:09
'''
import time
class MPU6050():

    def __init__(self, i2c, addr=0x68):
        self.iic = i2c
        self.addr = addr

        # http://www.51hei.com/bbs/dpj-136277-1.html
        # MPU掉电后所有设置清零
        i2c.writeto_mem(addr, 0x6B, bytearray([0b00000000])) # 使能温度
        i2c.writeto_mem(addr, 0x19, bytearray([0b00000000])) # 不分频
        i2c.writeto_mem(addr, 0x1A, bytearray([0b00000000])) # 关闭滤波器
        i2c.writeto_mem(addr, 0x1B, bytearray([0b00011000])) # gyro量程+-2000°/s
        i2c.writeto_mem(addr, 0x1C, bytearray([0b00001000])) # 加速度量程+-4g
        i2c.writeto_mem(addr, 0x23, bytearray([0b00000000])) # 关闭FIFO
        
    def get_raw_values(self):
        a = self.iic.readfrom_mem(self.addr, 0x3B, 14)
        return a

    def get_ints(self):
        b = self.get_raw_values()
        c = []
        for i in b:
            c.append(i)
        return c

    def bytes_toint(self, firstbyte, secondbyte):
        if not firstbyte & 0x80:
            return firstbyte << 8 | secondbyte
        return - (((firstbyte ^ 255) << 8) | (secondbyte ^ 255) + 1)

    def get_values(self):
        raw_ints = self.get_raw_values()
        vals = {}
        vals["time"] = time.ticks_us()
        vals["ax"] = self.bytes_toint(raw_ints[0], raw_ints[1])
        vals["ay"] = self.bytes_toint(raw_ints[2], raw_ints[3])
        vals["az"] = self.bytes_toint(raw_ints[4], raw_ints[5])
        vals["tmp"] = self.bytes_toint(raw_ints[6], raw_ints[7]) # / 340.00 + 36.53
        vals["gx"] = self.bytes_toint(raw_ints[8], raw_ints[9])
        vals["gy"] = self.bytes_toint(raw_ints[10], raw_ints[11])
        vals["gz"] = self.bytes_toint(raw_ints[12], raw_ints[13])
        return vals  # returned in range of Int16
        # -32768 to 32767
    