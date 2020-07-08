class speedadjust():
    
    def __init__(sefl):
        self.speedadjust = 1.0

    def speedincrease(self):
        global speedadjust
        speedadjust = 2.0
        print("In speedincrease",speedadjust)



