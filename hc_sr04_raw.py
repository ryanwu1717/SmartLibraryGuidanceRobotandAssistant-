import RPi.GPIO as GPIO
import time
 
TRIGGER_PIN = 25
ECHO_PIN = 8
 
GPIO.setmode(GPIO.BCM)
GPIO.setup(TRIGGER_PIN, GPIO.OUT)
GPIO.setup(ECHO_PIN, GPIO.IN)
 
GPIO.output(TRIGGER_PIN, GPIO.LOW)
time.sleep(1)
 
try:
    print('Ctrl-C')
    while True:
        GPIO.output(TRIGGER_PIN, GPIO.HIGH)
        time.sleep(0.00001)
        GPIO.output(TRIGGER_PIN, GPIO.LOW)
        while GPIO.input(ECHO_PIN) == 0:
            start_time = time.time()
        while GPIO.input(ECHO_PIN) == 1:
            end_time = time.time()
        etime = end_time - start_time
        distance = 17150 * etime
        print('{:.1f}'.format(distance))
        time.sleep(5)
except KeyboardInterrupt:
    print('closeapp')
finally:
    GPIO.cleanup()
