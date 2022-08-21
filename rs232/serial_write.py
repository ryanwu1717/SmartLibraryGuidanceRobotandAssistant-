#!/usr/bin/env python


import time
import serial
import OpenLockernumber   
import sys

ser = serial.Serial(

	port='/dev/ttyS0',
	baudrate = 9600,
	parity=serial.PARITY_NONE,
	stopbits=serial.STOPBITS_ONE,
	bytesize=serial.EIGHTBITS,
	timeout=1
)

ser.write(OpenLockernumber.openlockernumber(int(sys.argv[1])))
