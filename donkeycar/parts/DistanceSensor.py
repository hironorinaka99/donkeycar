#
# DistanceSensor.py
# functions to get distance inputs from Ultrasound Part
#
# @rossmelbourne
#

import RPi.GPIO as GPIO
import time
import signal
import sys

# use Raspberry Pi board pin numbers
GPIO.setmode(GPIO.BCM)

# set GPIO Pins
pinTrigger = 18
pinEcho = 24
brakingDistance = 120
sleepTime = 0.1 # run sensor 10 times to second

class DistanceSensor():

    def __init__(self):

        self.distance = 0.00
        self.throttle = 0
        self.running = True
        self.mode = 'run_pilot'

        def close(signal, frame):
            print("\nTurning off ultrasonic distance detection...\n")
            GPIO.cleanup()
            sys.exit(0)

        signal.signal(signal.SIGINT, close)

        # set GPIO input and output channels
        GPIO.setup(pinTrigger, GPIO.OUT)
        GPIO.setup(pinEcho, GPIO.IN)

        return

    def run_threaded(self):
        return self.distance, self.throttle, self.mode

    def run(self):
        raise Exception("We expect DistanceSensor Part to be run with the threaded=True argument.")
        return None, None, None

    def update(self):
        print("In DistanceSensor update")
        while self.running:
            try: #Nakagawa
                self.listenToDistanceSensor()
            except: #Nakagawa
                pass



    def listenToDistanceSensor(self):
        # set Trigger to HIGH
        GPIO.output(pinTrigger, True)
        # set Trigger after 0.01ms to LOW
        time.sleep(0.00001)
        GPIO.output(pinTrigger, False)

        startTime = time.time()
        stopTime = time.time()

        # save start time
        while 0 == GPIO.input(pinEcho):
            startTime = time.time()

        # save time of arrival
        while 1 == GPIO.input(pinEcho):
            stopTime = time.time()

        # time difference between start and arrival
        TimeElapsed = stopTime - startTime
        # multiply with the sonic speed (34300 cm/s)
        # and divide by 2, because there and back
        self.distance = (TimeElapsed * 34300) / 2

        # print ("Distance is: %.1f cm" % self.distance)

        if self.distance < brakingDistance:
            print ("Distance is: %.1f cm" % self.distance + " stopping...")
            self.throttle = 0
            self.mode = 'user'
        else:
            self.mode = 'run_pilot'

        time.sleep(sleepTime) 

    def shutdown(self):
        self.distance = 0
        self.running = False ;
        #time.sleep(0.5) #Add Nakagawa
        return
