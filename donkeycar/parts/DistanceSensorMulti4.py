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

sleepTime = 0.00002 # 元は、すべて0.0001 一度3分の1に変更
DMSread = 0
DMSlisten = 0

class DistanceSensorMulti4():
    def __init__(self):
        self.distanceLL = 0.00
        self.distanceL = 0.00
        self.distanceC = 0.00
        self.distanceR = 0.00
        self.distanceRR = 0.00

        self.prev_distanceLL = 0.00  #前回測定値を保持するため
        self.prev_distanceL  = 0.00
        self.prev_distanceC  = 0.00
        self.prev_distanceR  = 0.00
        self.prev_distanceRR = 0.00

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

    def run_threaded(self): #呼び出されて距離を返す
        global DMSread
        global DMSlisten

        #print("runThreaded")
        #DMSread +=1
        #print("DMS read %8d" % DMSread + "DMS listen %8d" % DMSlisten)
        if self.distanceLL < 2 or self.distanceL < 2 or self.distanceC < 2 or self.distanceR < 2 or self.distanceRR < 2: #エラー処理　1未満はエラー表示
            #print("DMS sensor error!!")

            print("DMS sensor error!!　2より小さい値は120cmで返却") #1より小さければ120
            print ("LL: %.1f cm" % self.distanceLL +"L: %.1f cm" % self.distanceL +"  " "C: %.1f cm" % self.distanceC + "  " "R: %.1f cm" % self.distanceR + "  " "RR: %.1f cm" % self.distanceRR) 
            if self.distanceLL < 2: #2より小さければ120
                self.distanceLL = 120
            if self.distanceL < 2:
                self.distanceL = 120
            if self.distanceC < 2:
                self.distanceC = 120
            if self.distanceR < 2:
                self.distanceR = 120
            if self.distanceRR < 2:
                self.distanceRR = 120
            
            print ("修正後 LL: %.1f cm" % self.distanceLL +"L: %.1f cm" % self.distanceL +"  " "C: %.1f cm" % self.distanceC + "  " "R: %.1f cm" % self.distanceR + "  " "RR: %.1f cm" % self.distanceRR) 



        #print ("Prev LL: %3.1f cm" % self.prev_distanceLL +"L: %3.1f cm" % self.prev_distanceL +"  " "C: %3.1f cm" % self.prev_distanceC + "  " "R: %3.1f cm" % self.prev_distanceR + "  " "RR: %3.1f cm" % self.prev_distanceRR)
        #print ("     LL: %5.1f cm" % self.distanceLL +"L: %5.1f cm" % self.distanceL +"  " "C: %5.1f cm" % self.distanceC + "  " "R: %5.1f cm" % self.distanceR + "  " "RR: %5.1f cm" % self.distanceRR)
        return self.distanceLL, self.distanceL, self.distanceC, self.distanceR, self.distanceRR, self.prev_distanceLL, self.prev_distanceL, self.prev_distanceC, self.prev_distanceR, self.prev_distanceRR

    def run(self):
        raise Exception("We expect DistanceSensor Part to be run with the threaded=True argument.")
        return None, None, None, None, None, None, None, None

    def update(self): #距離測定を繰り返す
        global DMSlisten
        print("In DistanceSensorMulti update")
        while self.running:
            #DMSlisten +=1
            self.listenToDistanceSensor()
            #self.listenToDistanceSensor(self.distanceLL, self.distanceL, self.distanceC, self.distanceR, self.distanceRR)
            #print ("Update LL: %.1f cm" % self.distanceLL +"L: %.1f cm" % self.distanceL +"  " "C: %.1f cm" % self.distanceC + "  " "R: %.1f cm" % self.distanceR + "  " "RR: %.1f cm" % self.distanceRR)
            #print ("UpdatePrev LL: %.1f cm" % self.prev_distanceLL +"L: %.1f cm" % self.prev_distanceL +"  " "C: %.1f cm" % self.prev_distanceC + "  " "R: %.1f cm" % self.prev_distanceR + "  " "RR: %.1f cm" % self.prev_distanceRR)

    def listenToDistanceSensor(self):    
    #def listenToDistanceSensor(self, distanceLL, distanceL, distanceC, distanceR, distanceRR):
        #前回測定値を保持
        self.prev_distanceLL = self.distanceLL
        #print("self.prev_distanceLL" + str(self.prev_distanceLL) + "self.distanceLL" + str(self.distanceLL))
        self.prev_distanceL = self.distanceL
        self.prev_distanceC = self.distanceC
        self.prev_distanceR = self.distanceR
        self.prev_distanceRR = self.distanceRR


        time.sleep(0.00002)
        # set Trigger to HIGH  DistanceSensorLeftLeft
        GPIO.output(pinTriggerLL, True)
        # set Trigger after 0.01ms to LOW
        time.sleep(0.00001)
        GPIO.output(pinTriggerLL, False)
        time.sleep(0.000001) #Centerで必要だったので追加

        startTimeLL = time.time()
        stopTimeLL = time.time()

        # save start time
        temp_count =0 
        while 0 == GPIO.input(pinEchoLL) and temp_count < 1000:
            temp_count +=1
            startTimeLL = time.time()

        # save time of arrival
        temp_count =0 
        while 1 == GPIO.input(pinEchoLL) and temp_count < 2000: #真横は70cmまで
            temp_count +=1
            stopTimeLL = time.time()

        TimeElapsedLL = stopTimeLL - startTimeLL #左回りコースのため、左側のみ広い範囲で取る
        if (TimeElapsedLL * 34300) / 2 < 70: #70未満は生値、それ以上は70として返す
            self.distanceLL = (TimeElapsedLL * 34300) / 2
        else: 
            self.distanceLL = 70

        time.sleep(0.00002)
        # set Trigger to HIGH  DistanceSensorCenter
        GPIO.output(pinTriggerC, True)
        time.sleep(0.00001)
        # set Trigger after 0.01ms to LOW
        GPIO.output(pinTriggerC, False)
        time.sleep(0.000001) #不要なはずー必要でした

        startTimeC = time.time()
        stopTimeC = time.time()

        # save start time
        temp_count = 0
        while 0 == GPIO.input(pinEchoC) and temp_count < 1000:
            temp_count +=1
            startTimeC = time.time()

        # save time of arrival
        temp_count =0 
        while 1 == GPIO.input(pinEchoC) and temp_count < 6000: #rp4だと、200cm 6000ループ　rp3だと3000
            temp_count +=1
            stopTimeC = time.time()

        TimeElapsedC = stopTimeC - startTimeC
        if (TimeElapsedC * 34300) / 2 < 200: #200未満は生値、それ以上は200として返す
            self.distanceC = (TimeElapsedC * 34300) / 2
        else: 
            self.distanceC = 200

        time.sleep(0.00002)
        # set Trigger to HIGH  DistanceSensorRightRight
        GPIO.output(pinTriggerRR, True)
        # set Trigger after 0.01ms to LOW
        time.sleep(0.00001)
        GPIO.output(pinTriggerRR, False)
        time.sleep(0.000001) #Centerで必要だったので追加

        startTimeRR = time.time()
        stopTimeRR = time.time()

        # save start time
        temp_count = 0
        while 0 == GPIO.input(pinEchoRR) and temp_count < 1000:
            temp_count +=1
            startTimeRR = time.time()

        # save time of arrival
        temp_count =0 
        while 1 == GPIO.input(pinEchoRR) and temp_count < 2000:
            temp_count +=1
            stopTimeRR = time.time()
            
        TimeElapsedRR = stopTimeRR - startTimeRR
        if (TimeElapsedRR * 34300) / 2 < 70: #70未満は生値、それ以上は70として返す
            self.distanceRR = (TimeElapsedRR * 34300) / 2
        else: 
            self.distanceRR = 70

        time.sleep(0.00002)
        # set Trigger to HIGH  DistanceSensorLeft
        GPIO.output(pinTriggerL, True)
        # set Trigger after 0.01ms to LOW
        time.sleep(0.00001)
        GPIO.output(pinTriggerL, False)
        time.sleep(0.000001) #Centerで必要だったので追加

        startTimeL = time.time()
        stopTimeL = time.time()

        # save start time
        temp_count =0 
        while 0 == GPIO.input(pinEchoL) and temp_count < 1000:
            temp_count +=1
            startTimeL = time.time()

        # save time of arrival
        temp_count =0 
        while 1 == GPIO.input(pinEchoL) and temp_count < 4000: #140cmの時間まで待つ
            temp_count +=1
            stopTimeL = time.time()

        TimeElapsedL = stopTimeL - startTimeL
        if (TimeElapsedL * 34300) / 2 < 140: #140未満は生値、それ以上は140として返す
            self.distanceL = (TimeElapsedL * 34300) / 2
        else: 
            self.distanceL = 140

        time.sleep(0.00002)
        # set Trigger to HIGH  DistanceSensorRight
        GPIO.output(pinTriggerR, True)
        # set Trigger after 0.01ms to LOW
        time.sleep(0.00001)
        GPIO.output(pinTriggerR, False)
        time.sleep(0.000001) #Centerで必要だったので追加

        startTimeR = time.time()
        stopTimeR = time.time()

        # save start time
        temp_count = 0
        while 0 == GPIO.input(pinEchoR) and temp_count < 1000:
            temp_count +=1
            startTimeR = time.time()

        # save time of arrival
        temp_count =0 
        while 1 == GPIO.input(pinEchoR) and temp_count < 4000:
            temp_count +=1
            stopTimeR = time.time()
            
        TimeElapsedR = stopTimeR - startTimeR
        if (TimeElapsedR * 34300) / 2 < 140: #140未満は生値、それ以上は140として返す
            self.distanceR = (TimeElapsedR * 34300) / 2
        else: 
            self.distanceR = 140


        time.sleep(sleepTime) 


    def shutdown(self):
        self.distanceLL = 0
        self.distanceL = 0
        self.distanceC = 0
        self.distanceR = 0
        self.distanceRR = 0
        self.running = False ;


        return
