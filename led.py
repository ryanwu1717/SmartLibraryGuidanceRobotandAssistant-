import RPi.GPIO as GPIO
import time
 
GPIO_PIN = 4
GPIO.setmode(GPIO.BCM)
GPIO.setup(GPIO_PIN, GPIO.OUT)
 
try:
    print('ctrl-c to exit')
    while True:
        print('LED open')
        GPIO.output(GPIO_PIN, GPIO.HIGH)
        time.sleep(1)
        print('LED close')
        GPIO.output(GPIO_PIN, GPIO.LOW)
        time.sleep(1)
except KeyboardInterrupt:
    print('Exit')
finally:
    GPIO.cleanup()
