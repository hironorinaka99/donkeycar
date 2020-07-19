class speedadjustclass():
    def __init__(self):
        self.speedadjust = 1.0
        return

    def speedincrease(self):
        self.speedadjust = round(min(2.5, self.speedadjust + 0.1), 2)
        print("In speedincrease",self.speedadjust)

    def speeddecrease(self):
        self.speedadjust = round(max(0.5, self.speedadjust - 0.1), 2)
        print("In speeddecrease",self.speedadjust)

    def run(self):
        return self.speedadjust





