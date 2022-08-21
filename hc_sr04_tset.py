import RPi.GPIO as GPIO

GPIO.setmode(GPIO.BCM)

GPIO.setup(23, GPIO.OUT)
GPIO.setup(24, GPIO.OUT)
GPIO.setup(16, GPIO.OUT)
GPIO.setup(20, GPIO.OUT)

while True:
    ch = input("f b l r q ")
    if ch == 'q':
       GPIO.output(17, False)
       GPIO.output(18, False)
       GPIO.output(22, False)
       GPIO.output(23, False)
       break
    if ch == 'f':
       GPIO.output(23, False)
       GPIO.output(24, True)
       GPIO.output(16, False)
       GPIO.output(20, True)
    if ch == 'b':
       GPIO.output(23, True)
       GPIO.output(24, False)
       GPIO.output(16, True)
       GPIO.output(20, False)
    if ch == 'r':
       GPIO.output(17, False)
       GPIO.output(18, True)
       GPIO.output(22, False)
       GPIO.output(23, False)
    if ch == 'l':
       GPIO.output(17, False)
       GPIO.output(18, False)
       GPIO.output(22, False)
       GPIO.output(23, True)
