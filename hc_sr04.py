#Libraries
from hcsr04sensor import sensor
import time
import RPi.GPIO as GPIO

#GPIO Mode (BOARD / BCM)
GPIO.setmode(GPIO.BCM)

#set GPIO Pins
GPIO_TRIGGER = 25
GPIO_ECHO = 8
GPIO_ENA = 27
GPIO_PosR = 18
GPIO_NegR = 17
GPIO_PosL = 23
GPIO_NegL = 22
GPIO_ENB = 24
distance = 0

#set GPIO direction (IN / OUT)
GPIO.setup(GPIO_TRIGGER, GPIO.OUT)
GPIO.setup(GPIO_ECHO, GPIO.IN)
GPIO.setup(GPIO_PosR, GPIO.OUT)
GPIO.setup(GPIO_NegR, GPIO.OUT)
GPIO.setup(GPIO_PosL, GPIO.OUT)
GPIO.setup(GPIO_NegL, GPIO.OUT)
GPIO.setup(GPIO_ENA, GPIO.OUT)
GPIO.setup(GPIO_ENB, GPIO.OUT)

#set GPIO PWM frenquence
p = GPIO.PWM(GPIO_ENA, 1000)
q = GPIO.PWM(GPIO_ENB, 1000)

#funtion code
def stop():
    print('stop action')
    GPIO.output(GPIO_PosR, False)
    GPIO.output(GPIO_NegR, False)
    GPIO.output(GPIO_PosL, False)
    GPIO.output(GPIO_NegL, False)
    print('stop over')

def forward(a):
    print('forward action')
    GPIO.output(GPIO_PosR, False)
    GPIO.output(GPIO_NegR, True)
    GPIO.output(GPIO_PosL, False)
    GPIO.output(GPIO_NegL, True)
    if(a == 0):
        time.sleep(0.75)
    else:
        time.sleep(a)

    print('forward stop')
    stop()

def backward(a):
    print('backward action')
    GPIO.output(GPIO_PosR, True)
    GPIO.output(GPIO_NegR, False)
    GPIO.output(GPIO_PosL, True)
    GPIO.output(GPIO_NegL, False)
    if(a == 0):
        time.sleep(0.75)
    else:
        time.sleep(a)
    print('backward stop')
    stop()

def turnRight(a):
    print('action')
    GPIO.output(GPIO_PosR, False)
    GPIO.output(GPIO_NegR, False)
    GPIO.output(GPIO_PosL, False)
    GPIO.output(GPIO_NegL, True)
    if(a == 0):
        time.sleep(1)
    else:
        time.sleep(a)
    stop()
    print('over')
def turnLeft(a):
    print('actionL')
    GPIO.output(GPIO_PosR, False)
    GPIO.output(GPIO_NegR, True)
    GPIO.output(GPIO_PosL, False)
    GPIO.output(GPIO_NegL, False)
    if(a == 0):
        time.sleep(1)
    else:
        time.sleep(a)
    stop()
    print('overL')

def turnbackLeft(a):
    print('actionL')
    GPIO.output(GPIO_PosR, True)
    GPIO.output(GPIO_NegR, False)
    GPIO.output(GPIO_PosL, False)
    GPIO.output(GPIO_NegL, False)
    if(a==0):
        time.sleep(1)
    else:
        time.sleep(a)
    stop()
    print('overL')
def d1():
    sr04 = sensor.Measurement(GPIO_TRIGGER, GPIO_ECHO)
#    print(sr04)
    raw_measurement = sr04.raw_distance()
#    print(raw_measurement)
    distance = sr04.distance_metric(raw_measurement)
#    print(distance)
    print('distance is {:.1f} cm'.format(distance))
    #time.sleep(1)
    return distance

#main

def fix():
    # turnbackLeft(1.0)
    # turnLeft(0)
    backward(0)

    forward(0)


try:
   # distance = d1()
    print(distance)
    p.start(65)
    q.start(65)
    print('ctrl-c to close \n')
    print('the default motor is Forward\n')
    tmpLocation = [0,0]
    gole=[6,6]
    moveX = gole[0] - tmpLocation[0] 
    moveY = gole[1] - tmpLocation[1] 
    while True:
        print(distance)
        distance = d1()
        fix()
        for k in range(0, moveX):
            forward(0)
        if(moveY>0):
            turnLeft(0)
        elif (moveY<0):
            turnRight(0)
            # moveY*=1
        fix()
        for k in range(0, moveY):
            forward(0)
        # turnbackLeft(1.0)
        # turnLeft(0.8)
        # forward(2.5)
        # turnbackLeft(1.0)
        # turnLeft(0.9)
        # forward(7.5)
        break


        if distance >= 200:
            print(distance)
            p.ChangeDutyCycle(100)
            q.ChangeDutyCycle(100)
            forward()
#            d1()
        elif distance < 200 and distance >= 130:
            print(distance)
            p.ChangeDutyCycle(85)
            q.ChangeDutyCycle(85)
            forward()
#            d1()
        elif distance < 130 and distance >= 70:
            print(distance)
            p.ChangeDutyCycle(65)
            q.ChangeDutyCycle(65)
            forward()
#            d1()  
        else:
            stop()
            print(distance)
            p.ChangeDutyCycle(70)
            q.ChangeDutyCycle(70)
            backward()
            time.sleep(2)
            p.ChangeDutyCycle(65)
            q.ChangeDutyCycle(65)
            turnRight()
            gap1 = d1()
            turnLeft()
            turnLeft()
            gap2 = d1()
            if gap1 > gap2:
               turnRight()
               turnRight()
#            d1()

except KeyboardInterrupt:
    print('closetheapp')
except UnboundLocalError as e:
    print(e)
    print('Error1')
except SystemError:
    print('Error2')
finally:
    GPIO.cleanup()
