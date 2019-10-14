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
pinTrigger1 = 18
pinEcho1 = 24
brakingDistance1 = 20

pinTrigger2 = 23
pinEcho2 = 25
brakingDistance2 = 20

sleepTime = 0.1 # run sensor 5 times to second

class DistanceSensorMulti():

    def __init__(self):

        self.distance1 = 0.00
        self.distance2 = 0.00
        self.throttle = 0
        self.running = True
        self.mode = 'run_pilot'

        def close(signal, frame):
            print("\nTurning off ultrasonic distance detection...\n")
            GPIO.cleanup()
            sys.exit(0)

        signal.signal(signal.SIGINT, close)

        # set GPIO input and output channels
        GPIO.setup(pinTrigger1, GPIO.OUT)
        GPIO.setup(pinEcho1, GPIO.IN)
        GPIO.setup(pinTrigger2, GPIO.OUT)
        GPIO.setup(pinEcho2, GPIO.IN)

        return

    def run_threaded(self):
        print ("Dis1 is: %.1f cm" % self.distance1 +"  " "Dis2 is: %.1f cm" % self.distance2)
        return self.distance1, self.distance2

    def run(self):
        raise Exception("We expect DistanceSensor Part to be run with the threaded=True argument.")
        #return None, None, None, None
        return self.distance1, self.distance2

    def update(self):
        print("In DistanceSensorMulti update")
        while self.running:
            try: #Nakagawa
                self.listenToDistanceSensor()
            except: #Nakagawa
                pass

    def listenToDistanceSensor(self):
        # set Trigger to HIGH  DistanceSensor1
        GPIO.output(pinTrigger1, True)
        # set Trigger after 0.01ms to LOW
        time.sleep(0.00001)
        GPIO.output(pinTrigger1, False)

        startTime1 = time.time()
        stopTime1 = time.time()

        # save start time
        while 0 == GPIO.input(pinEcho1):
            startTime1 = time.time()

        # save time of arrival
        while 1 == GPIO.input(pinEcho1):
            stopTime1 = time.time()

        # time difference between start and arrival
        TimeElapsed1 = stopTime1 - startTime1
        # multiply with the sonic speed (34300 cm/s)
        # and divide by 2, because there and back
        self.distance1 = (TimeElapsed1 * 34300) / 2

        time.sleep(0.001) #混信防止

        # set Trigger to HIGH  DistanceSensor2
        GPIO.output(pinTrigger2, True)
        # set Trigger after 0.01ms to LOW
        time.sleep(0.00001)
        GPIO.output(pinTrigger2, False)

        startTime2 = time.time()
        stopTime2 = time.time()

        # save start time
        while 0 == GPIO.input(pinEcho2):
            startTime2 = time.time()

        # save time of arrival
        while 1 == GPIO.input(pinEcho2):
            stopTime2 = time.time()

        # time difference between start and arrival
        TimeElapsed2 = stopTime2 - startTime2
        # multiply with the sonic speed (34300 cm/s)
        # and divide by 2, because there and back
        self.distance2 = (TimeElapsed2 * 34300) / 2

        """
        if self.distance1 < brakingDistance1 or self.distance2 < brakingDistance2:
            print ("Dis1 is: %.1f cm" % self.distance1 +"  " "Dis2 is: %.1f cm" % self.distance2 + " stopping...")
            self.throttle = 0
            self.mode = 'user'
        else:
            self.mode = 'run_pilot'
        """

        time.sleep(sleepTime) 

    def shutdown(self):
        self.distance1 = 0
        self.distance2 = 0
        self.running = False ;
        return
