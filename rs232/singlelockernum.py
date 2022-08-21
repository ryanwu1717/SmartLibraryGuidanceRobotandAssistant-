#!/usr/bin/env python


import time
import serial
import sys

ser = serial.Serial(

        port='/dev/ttyS0',
        baudrate = 9600,
        parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE,
        bytesize=serial.EIGHTBITS,
        timeout=1
)
a = bytearray([170,0,1,0,0,0,0,85])




a[1]=10
a[5]=12
a[6]=7

ser.write(a)
print("lockernumber ="+str(a[1]))
print("locker = "+str(a[5]))
print("num = "+str(a[6]))
