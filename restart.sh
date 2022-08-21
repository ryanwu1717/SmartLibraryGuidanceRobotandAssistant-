#!/bin/sh
ps -ef | grep RFIDRead.py | grep -v grep | awk '{print $2}' | xargs sudo kill
/usr/bin/python /home/pi/rs232/RFIDRead.py
