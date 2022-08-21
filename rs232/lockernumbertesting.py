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

a = bytearray([170,15,1,0,0,1,0,85])

for locker in range(1,13):

	for num in range(0,16): 
		a[5]=locker
		a[6]=num
		ser.write(a)
		print("locker = "+str(locker))
		print("num = "+str(num))
		time.sleep(2)

