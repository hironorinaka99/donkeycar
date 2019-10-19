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
"""
pinTriggerL = 18
pinEchoL = 23
brakingDistanceL = 20

pinTriggerC = 25
pinEchoC = 24
brakingDistanceC = 20

pinTriggerR = 8
pinEchoR = 7
brakingDistanceR = 20
"""
pinTriggerL = 18 #19　
pinEchoL =  23 #20
brakingDistanceL = 20

pinTriggerC = 25
pinEchoC = 24
brakingDistanceC = 20

pinTriggerR = 26
pinEchoR = 21
brakingDistanceR = 20

sleepTime = 0.1 # run sensor 5 times to second

class DistanceSensorMulti2():

    def __init__(self):

        self.distanceL = 0.00
        self.distanceC = 0.00
        self.distanceR = 0.00

        self.throttle = 0
        self.running = True
        #self.mode = 'run_pilot'

        def close(signal, frame):
            print("\nTurning off ultrasonic distance detection...\n")
            GPIO.cleanup()
            sys.exit(0)

        signal.signal(signal.SIGINT, close)

        # set GPIO input and output channels
        GPIO.setup(pinTriggerL, GPIO.OUT)
        GPIO.setup(pinEchoL, GPIO.IN)
        GPIO.setup(pinTriggerC, GPIO.OUT)
        GPIO.setup(pinEchoC, GPIO.IN)
        GPIO.setup(pinTriggerR, GPIO.OUT)
        GPIO.setup(pinEchoR, GPIO.IN)

        return

    def run_threaded(self):
        #print ("DisL is: %.1f cm" % self.distanceL +"  " "DisC is: %.1f cm" % self.distanceC + "  " "DisR is: %.1f cm" % self.distanceR)
        return self.distanceL, self.distanceC, self.distanceR

    def run(self):
        raise Exception("We expect DistanceSensor Part to be run with the threaded=True argument.")
        #return None, None, None, None
        return self.distanceL, self.distanceC, self.distanceR

    def update(self):
        print("In DistanceSensorMulti update")
        while self.running:
            try: #Nakagawa
                self.listenToDistanceSensor()
                #print ("Update  DisL is: %.1f cm" % self.distanceL +"  " "DisC is: %.1f cm" % self.distanceC + "  " "DisR is: %.1f cm" % self.distanceR)

            except: #Nakagawa
                pass

    def listenToDistanceSensor(self):
        # set Trigger to HIGH  DistanceSensorLeft
        GPIO.output(pinTriggerL, True)
        # set Trigger after 0.01ms to LOW
        time.sleep(0.00001)
        GPIO.output(pinTriggerL, False)

        startTimeL = time.time()
        stopTimeL = time.time()

        # save start time
        temp_count =0 
        while 0 == GPIO.input(pinEchoL) and temp_count < 1000:
            temp_count +=1
            startTimeL = time.time()

        # save time of arrival
        temp_count =0 
        while 1 == GPIO.input(pinEchoL) and temp_count < 1000:
            temp_count +=1
            stopTimeL = time.time()


        # time difference between start and arrival
        TimeElapsedL = stopTimeL - startTimeL
        # multiply with the sonic speed (34300 cm/s)
        # and divide by 2, because there and back
        self.distanceL = (TimeElapsedL * 34300) / 2

        #time.sleep(0.001) #混信防止

        # set Trigger to HIGH  DistanceSensorCenter
        GPIO.output(pinTriggerC, True)
        # set Trigger after 0.01ms to LOW
        time.sleep(0.00001)
        GPIO.output(pinTriggerC, False)

        startTimeC = time.time()
        stopTimeC = time.time()

        # save start time
        temp_count = 0
        while 0 == GPIO.input(pinEchoC) and temp_count < 1000:
            temp_count +=1
            startTimeC = time.time()

        # save time of arrival
        temp_count =0 
        while 1 == GPIO.input(pinEchoC) and temp_count < 1000:
            temp_count +=1
            stopTimeC = time.time()


        # time difference between start and arrival
        TimeElapsedC = stopTimeC - startTimeC
        # multiply with the sonic speed (34300 cm/s)
        # and divide by 2, because there and back
        self.distanceC = (TimeElapsedC * 34300) / 2

        #time.sleep(0.001) #混信防止

        # set Trigger to HIGH  DistanceSensorRight
        GPIO.output(pinTriggerR, True)
        # set Trigger after 0.01ms to LOW
        time.sleep(0.00001)
        GPIO.output(pinTriggerR, False)

        startTimeR = time.time()
        stopTimeR = time.time()

        # save start time
        temp_count = 0
        while 0 == GPIO.input(pinEchoR) and temp_count < 1000:
            temp_count +=1
            startTimeR = time.time()

        # save time of arrival
        temp_count =0 
        while 1 == GPIO.input(pinEchoR) and temp_count < 1000:
            temp_count +=1
            stopTimeR = time.time()
            

        # time difference between start and arrival
        TimeElapsedR = stopTimeR - startTimeR
        # multiply with the sonic speed (34300 cm/s)
        # and divide by 2, because there and back
        self.distanceR = (TimeElapsedR * 34300) / 2
        time.sleep(sleepTime) 

    def shutdown(self):
        self.distanceL = 0
        self.distanceC = 0
        self.distanceR = 0

        self.running = False ;
        return
