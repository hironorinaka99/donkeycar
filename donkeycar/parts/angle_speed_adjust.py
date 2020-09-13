class angle_speed_adjustclass(): #ステアリングを切ったときに、速度を調整する
    def __init__(self):
        self.angle_speed_adjust = 1.0
        return

    def angleincrease(self):
        self.angle_speed_adjust = round(min(1.5, self.angle_speed_adjust + 0.05), 2)
        print("In angleincrease",self.angle_speed_adjust)

    def angledecrease(self):
        self.angle_speed_adjust = round(max(0.5, self.angle_speed_adjust - 0.05), 2)
        print("In angledecrease",self.angle_speed_adjust)

    def run(self):
        return self.angle_speed_adjust