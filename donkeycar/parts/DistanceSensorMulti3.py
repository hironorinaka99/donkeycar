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
pinTriggerLL = 19
pinEchoLL = 20
brakingDistanceLL = 10

pinTriggerL = 18
pinEchoL = 23
brakingDistanceL = 20

pinTriggerC = 25
pinEchoC = 24
brakingDistanceC = 20

pinTriggerR = 8
pinEchoR = 7
brakingDistanceR = 20

pinTriggerRR = 26
pinEchoRR = 21
brakingDistanceRR = 10

sleepTime = 0.1 # run sensor 10 times to second

class DistanceSensorMulti3():

    def __init__(self):

        self.distanceLL = 0.00
        self.distanceL = 0.00
        self.distanceC = 0.00
        self.distanceR = 0.00
        self.distanceRR = 0.00

        self.throttle = 0
        self.running = True
        #self.mode = 'run_pilot'

        def close(signal, frame):
            print("\nTurning off ultrasonic distance detection...\n")
            GPIO.cleanup()
            sys.exit(0)

        signal.signal(signal.SIGINT, close)

        # set GPIO input and output channels
        GPIO.setup(pinTriggerLL, GPIO.OUT)
        GPIO.setup(pinEchoLL, GPIO.IN)
        GPIO.setup(pinTriggerL, GPIO.OUT)
        GPIO.setup(pinEchoL, GPIO.IN)
        GPIO.setup(pinTriggerC, GPIO.OUT)
        GPIO.setup(pinEchoC, GPIO.IN)
        GPIO.setup(pinTriggerR, GPIO.OUT)
        GPIO.setup(pinEchoR, GPIO.IN)
        GPIO.setup(pinTriggerRR, GPIO.OUT)
        GPIO.setup(pinEchoRR, GPIO.IN)

        return

    def run_threaded(self):
        if self.distanceLL < 0 or self.distanceL < 0 or self.distanceC < 0 or self.distanceR < 0 or self.distanceRR < 0: #エラー処理　マイナス値はエラー表示
            print("DMS sensor error!!")
            print ("LL: %.1f cm" % self.distanceLL +"L: %.1f cm" % self.distanceL +"  " "C: %.1f cm" % self.distanceC + "  " "R: %.1f cm" % self.distanceR + "  " "RR: %.1f cm" % self.distanceRR) 

        print ("LL: %.1f cm" % self.distanceLL +"L: %.1f cm" % self.distanceL +"  " "C: %.1f cm" % self.distanceC + "  " "R: %.1f cm" % self.distanceR + "  " "RR: %.1f cm" % self.distanceRR)
        return self.distanceLL, self.distanceL, self.distanceC, self.distanceR, self.distanceRR

    def run(self):
        raise Exception("We expect DistanceSensor Part to be run with the threaded=True argument.")
        return None, None, None, None

    def update(self):
        print("In DistanceSensorMulti update")
        while self.running:
            try: #Nakagawa
                self.listenToDistanceSensor()
                #print ("Update LL: %.1f cm" % self.distanceLL +"L: %.1f cm" % self.distanceL +"  " "C: %.1f cm" % self.distanceC + "  " "R: %.1f cm" % self.distanceR + "  " "RR: %.1f cm" % self.distanceRR)
        
            except: #Nakagawa
                pass

    def listenToDistanceSensor(self):
        # set Trigger to HIGH  DistanceSensorLeftLeft
        GPIO.output(pinTriggerLL, True)
        # set Trigger after 0.01ms to LOW
        time.sleep(0.00001)
        GPIO.output(pinTriggerLL, False)

        startTimeLL = time.time()
        stopTimeLL = time.time()

        # save start time
        temp_count =0 
        while 0 == GPIO.input(pinEchoLL) and temp_count < 1000:
            temp_count +=1
            startTimeLL = time.time()

        # save time of arrival
        temp_count =0 
        while 1 == GPIO.input(pinEchoLL) and temp_count < 1000:
            temp_count +=1
            stopTimeLL = time.time()

        TimeElapsedLL = stopTimeLL - startTimeLL
        self.distanceLL = (TimeElapsedLL * 34300) / 2

        # set Trigger to HIGH  DistanceSensorCenter
        GPIO.output(pinTriggerC, True)
        time.sleep(0.00001)
        # set Trigger after 0.01ms to LOW
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

        TimeElapsedC = stopTimeC - startTimeC
        self.distanceC = (TimeElapsedC * 34300) / 2

        # set Trigger to HIGH  DistanceSensorRightRight
        GPIO.output(pinTriggerRR, True)
        # set Trigger after 0.01ms to LOW
        time.sleep(0.00001)
        GPIO.output(pinTriggerRR, False)

        startTimeRR = time.time()
        stopTimeRR = time.time()

        # save start time
        temp_count = 0
        while 0 == GPIO.input(pinEchoRR) and temp_count < 1000:
            temp_count +=1
            startTimeRR = time.time()

        # save time of arrival
        temp_count =0 
        while 1 == GPIO.input(pinEchoRR) and temp_count < 1000:
            temp_count +=1
            stopTimeRR = time.time()
            
        TimeElapsedRR = stopTimeRR - startTimeRR
        self.distanceRR = (TimeElapsedRR * 34300) / 2

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

        TimeElapsedL = stopTimeL - startTimeL
        self.distanceL = (TimeElapsedL * 34300) / 2


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
            
        TimeElapsedR = stopTimeR - startTimeR
        self.distanceR = (TimeElapsedR * 34300) / 2


        time.sleep(sleepTime) 


    def shutdown(self):
        self.distanceLL = 0
        self.distanceL = 0
        self.distanceC = 0
        self.distanceR = 0
        self.distanceRR = 0

        self.running = False ;
        return
