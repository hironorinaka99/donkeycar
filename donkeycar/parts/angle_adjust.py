class angle_adjustclass(): #ステアリングの切れ角を調整する
    def __init__(self):
        self.angle_adjust = 1.0
        return

    def angleincrease(self):
        self.angle_adjust = round(min(2.0, self.angle_adjust + 0.05), 2)
        print("In angle_adjust increase",self.angle_adjust)

    def angledecrease(self):
        self.angle_adjust = round(max(0.5, self.angle_adjust - 0.05), 2)
        print("In angle_adjust increase",self.angle_adjust)

    def run(self):
        return self.angle_adjust