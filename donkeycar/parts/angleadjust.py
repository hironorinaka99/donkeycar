class angleadjustclass():
    def __init__(self):
        self.angleadjust = 1.0
        return

    def angleincrease(self):
        self.angleadjust = round(min(1.3, self.angleadjust + 0.05), 2)
        print("In angleincrease",self.angleadjust)

    def angledecrease(self):
        self.angleadjust = round(max(0.7, self.angleadjust - 0.05), 2)
        print("In angledecrease",self.angleadjust)

    def run(self):
        return self.angleadjust