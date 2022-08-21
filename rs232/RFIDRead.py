#!/usr/bin/env python

import RPi.GPIO as GPIO
import MFRC522
import signal
from datetime import datetime
import requests
import json
import os
import time

continue_reading = True

# Capture SIGINT for cleanup when the script is aborted
def end_read(signal,frame):
	global continue_reading
	print "Ctrl+C captured, ending read."
	continue_reading = False
    	GPIO.cleanup()

# Hook the SIGINT
signal.signal(signal.SIGINT, end_read)

# Create an object of the class MFRC522
MIFAREReader = MFRC522.MFRC522()

# Welcome message
print "Welcome to the MFRC522 data read example"
print "Press Ctrl-C to stop."

t1 = datetime.now()
t2 = datetime.now()
str_uid_old= ""

def openlockerapi(str_uid):
	try:
		r = requests.get("http://127.0.0.1/API/getbox/"+str_uid,timeout =3)
		if r.text=='null':
			r = requests.get("http://127.0.0.1/API/getbox/"+str_uid,timeout =3)
			return
	except requests.Timeout as e:
	    return
	json_str=r.text
	data=json.loads(json_str)
	if 'cLockerId' in data:	
		print (str(data['cLockerId'])) 
		os.system ("/usr/bin/python /home/pi/rs232/serial_write.py "+str(data['cLockerId']))

		if data['iAutoBookId']<0:
			iAutoBookId=''
		else:
			iAutoBookId=data['iAutoBookId']
		
		#IP port set 
		# cPort = '8017'

		# post_data = {'cIP':'140.127.41.248','cPort':cPort,'iAutoBookId':str(iAutoBookId),'cLockerId':str(data['cLockerId']),'dOpenDate':time.strftime("%Y-%m-%d %H:%M:%S", time.localtime()),'cInnercode':str_uid,'cAllowAccount':'lock@nknu','cPiMsg':'autocallback'}
		# url = 'http://pbws.nknu.edu.tw/WebAPI/SmartLock/PiOpenLockerThenInfoMeta'
		# r = requests.post(url,data=post_data,timeout =3)

while continue_reading:
    
    # Scan for cards    
    	(status,TagType) = MIFAREReader.MFRC522_Request(MIFAREReader.PICC_REQIDL)
    
	# Get the UID of the card
    	(status,uid) = MIFAREReader.MFRC522_Anticoll()

    # If we have the UID, continue
    	if status == MIFAREReader.MI_OK:

       		str_uid=str(hex(uid[0])).replace("0x", "").upper().zfill(2)+str(hex(uid[1])).replace("0x", "").upper().zfill(2)+str(hex(uid[2])).replace("0x", "").upper().zfill(2)+str(hex(uid[3])).replace("0x", "").upper().zfill(2) 
		str_uid=str_uid.replace("0x", "").upper() 
		#print "str_uid: %s" % str_uid
      		#print"str_uid_old: %s" % str_uid_old 
	
	
		if str_uid_old != str_uid:

            		print str_uid
            		str_uid_old = str_uid
            		t1 = datetime.now()
			openlockerapi(str_uid) 
        	else:
            		t2 = datetime.now()
            		delta = t2 - t1
           		#print "delta: %s" % delta.total_seconds() 
			
			if int(delta.total_seconds()) > int(1):  
                		print str_uid
				str_uid_old = str_uid
				t1 = datetime.now() 
				openlockerapi(str_uid) 
