#Libraries
import RPi.GPIO as GPIO
import time
 
#GPIO Mode (BOARD / BCM)
GPIO.setmode(GPIO.BCM)
 
#set GPIO Pins
GPIO_TRIGGER = 25
GPIO_ECHO = 8
GPIO_PosR = 21
GPIO_NegR = 20
GPIO_PosL = 16
GPIO_NegL = 12
 
#set GPIO direction (IN / OUT)
GPIO.setup(GPIO_TRIGGER, GPIO.OUT)
GPIO.setup(GPIO_ECHO, GPIO.IN)
GPIO.setup(GPIO_PosR, GPIO.OUT)
GPIO.setup(GPIO_NegR, GPIO.OUT)
GPIO.setup(GPIO_PosL, GPIO.OUT)
GPIO.setup(GPIO_NegL, GPIO.OUT)
 
def d1():
    # set Trigger to HIGH
    GPIO.output(GPIO_TRIGGER, True)
 
    # set Trigger after 0.01ms to LOW
    time.sleep(0.00001)
    GPIO.output(GPIO_TRIGGER, False)
 
    StartTime = time.time()
    StopTime = time.time()
 
    # save StartTime
    while GPIO.input(GPIO_ECHO) == 0:
        StartTime = time.time()
        print('Start is',StartTime)
    # save time of arrival
    while GPIO.input(GPIO_ECHO) == 1:
        StopTime = time.time()
        print('Stop is',StopTime)
    # time difference between start and arrival
    TimeElapsed = StopTime - StartTime
    print('TE is',TimeElapsed)
    # multiply with the sonic speed (34300 cm/s)
    # and divide by 2, because there and back
    distance = (TimeElapsed * 34300) / 2
    print(distance)
    GPIO.input(GPIO_ECHO) == 0
    return distance

def stop():
    GPIO.output(GPIO_PosR, False)
    GPIO.output(GPIO_NegR, False)
    GPIO.output(GPIO_PosL, False)
    GPIO.output(GPIO_NegL, False)

def forward():
    GPIO.output(GPIO_PosR, True)
    GPIO.output(GPIO_NegR, False)
    GPIO.output(GPIO_PosL, True)
    GPIO.output(GPIO_NegL, False)
    time.sleep(1)
    stop()

def backward():
    GPIO.output(GPIO_PosR, False)
    GPIO.output(GPIO_NegR, True)
    GPIO.output(GPIO_PosL, False)
    GPIO.output(GPIO_NegL, True)
    time.sleep(3)
    stop()

def turnRight():
    GPIO.output(GPIO_PosR, False)
    GPIO.output(GPIO_NegR, False)
    GPIO.output(GPIO_PosL, True)
    GPIO.output(GPIO_NegL, False)
    time.sleep(1)
    stop()

def turnLeft():
    GPIO.output(GPIO_PosR, True)
    GPIO.output(GPIO_NegR, False)
    GPIO.output(GPIO_PosL, False)
    GPIO.output(GPIO_NegL, False)
    time.sleep(1)
    stop()
try:
    while True:
        dist = d1()
        print ("Measured Distance = %.1f cm" % dist)
        time.sleep(3)

        if dist >= 3:
            forward()
        elif dist < 3:
            stop()
            backward()
    # Reset by pressing CTRL + C
except KeyboardInterrupt:
    print("Measurement stopped by User")
    GPIO.cleanup()
