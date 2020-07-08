class speedadjustclass():
    global speedadjust
    def __init__(self):
        return

    def speedincrease(self):
        global speedadjust
        speedadjust = 2.0
        print("In speedincrease",speedadjust)
        return speedadjust
    """
    def speeddecrease(self):
        global speedadjust
        speedadjust = 0.5
        print("In speeddecrease",speedadjust)
        return speedadjust
    """    
    def run(self):
        return speedadjust



