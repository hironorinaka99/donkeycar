class speedadjustclass():
    def __init__(self):
        self.speedadjust = 1.0
        return

    def speedincrease(self):
        self.speedadjust = 2.0
        print("In speedincrease",self.speedadjust)
        return speedadjust
    """
    def speeddecrease(self):
        global speedadjust
        speedadjust = 0.5
        print("In speeddecrease",speedadjust)
        return speedadjust
    """    
    def run(self):
        return self.speedadjust



