#!/usr/bin/env python3
import time
SENSOR_MPU6050 = 'mpu6050'
SENSOR_MPU9250 = 'mpu9250'

DLP_SETTING_DISABLED = 0
CONFIG_REGISTER = 0x1A

#初期値はダミー
magxmax = -100
magxmin = 100
magymax = -100
magymin = 100

class IMU:
    '''
    Installation:
    
    - MPU6050
    sudo apt install python3-smbus
    or
    sudo apt-get install i2c-tools libi2c-dev python-dev python3-dev
    git clone https://github.com/pimoroni/py-smbus.git
    cd py-smbus/library
    python setup.py build
    sudo python setup.py install

    pip install mpu6050-raspberrypi
    
    - MPU9250
    pip install mpu9250-jmdev
    
    '''

    #def __init__(self, addr=0x68, poll_delay=0.0166, sensor=SENSOR_MPU6050, dlp_setting=DLP_SETTING_DISABLED):
    def __init__(self, addr=0x68, poll_delay=0.0166, sensor=SENSOR_MPU6050, dlp_setting=DLP_SETTING_DISABLED):
        self.sensortype = sensor
        if self.sensortype == SENSOR_MPU6050:
            from mpu6050 import mpu6050 as MPU6050
            self.sensor = MPU6050(addr)
        
            if(dlp_setting > 0):
                self.sensor.bus.write_byte_data(self.sensor.address, CONFIG_REGISTER, dlp_setting)
        
        else:
            from mpu9250_jmdev.registers import AK8963_ADDRESS, GFS_1000, AFS_4G, AK8963_BIT_16, AK8963_MODE_C100HZ
            from mpu9250_jmdev.mpu_9250 import MPU9250
            
            self.sensor = MPU9250(
                address_ak=AK8963_ADDRESS,
                address_mpu_master=addr,  # In 0x68 Address
                address_mpu_slave=None,
                bus=1,
                gfs=GFS_1000,
                afs=AFS_4G,
                mfs=AK8963_BIT_16,
                mode=AK8963_MODE_C100HZ)
            
            if(dlp_setting > 0):
                self.sensor.writeSlave(CONFIG_REGISTER, dlp_setting)
            self.sensor.calibrateMPU6500()
            #self.sensor.calibrateAK8963() #Nakagawaつけたし

            self.sensor.configure()

        
        self.accel = { 'x' : 0., 'y' : 0., 'z' : 0. }
        self.gyro = { 'x' : 0., 'y' : 0., 'z' : 0. }
        self.mag = {'x': 0., 'y': 0., 'z': 0.}
        self.temp = 0.
        self.poll_delay = poll_delay
        self.on = True

    def update(self):
        while self.on:
            self.poll()
            time.sleep(self.poll_delay)
                
    def poll(self):
        try:
            if self.sensortype == SENSOR_MPU6050:
                self.accel, self.gyro, self.temp = self.sensor.get_all_data()
            else:
                from mpu9250_jmdev.registers import GRAVITY
                ret = self.sensor.getAllData()
                self.accel = { 'x' : ret[1] * GRAVITY, 'y' : ret[2] * GRAVITY, 'z' : ret[3] * GRAVITY }
                self.gyro = { 'x' : ret[4], 'y' : ret[5], 'z' : ret[6] }
                self.mag = { 'x' : ret[13], 'y' : ret[14], 'z' : ret[15] }
                self.temp = ret[16]
        except:
            print('failed to read imu!!')
            
    def run_threaded(self):
        global magxmax
        global magxmin
        global magymax
        global magymin
        if self.mag['x'] > magxmax:
            magxmax = self.mag['x']
        if self.mag['x'] < magxmin:
            magxmin = self.mag['x']
        if self.mag['y'] > magymax:
            magymax = self.mag['y']
        if self.mag['y'] < magymin:
            magymin = self.mag['y']
        
        x = self.mag['x'] - 43 #ズレ分を引く
        y = self.mag['y'] - 15

        #北 -1, 南 +1   西 -1 東 +1 
        xd = (x ** 2 / (x**2 + y**2)) ** 0.5 #方向成分を計算
        if x < 0:
            xd *= -1 #負の値を計算
        yd = (y ** 2 / (x**2 + y**2)) ** 0.5
        if y < 0:
            yd *= -1 #負の値を計算

        #print("IMU Mag X %5.1f  Y %5.1f Z %5.1f" % (self.mag['x'],self.mag['y'],self.mag['z']))
        #print("IMU Mag direction X %5.1f  Y %5.1f" % (xd,yd))
        
        return xd, yd, self.temp
        #return self.accel['x'], self.accel['y'], self.accel['z'], self.gyro['x'], self.gyro['y'], self.gyro['z'], self.temp

    def run(self):
        self.poll()

        global magxmax
        global magxmin
        global magymax
        global magymin
        if self.mag['x'] > magxmax:
            magxmax = self.mag['x']
        if self.mag['x'] < magxmin:
            magxmin = self.mag['x']
        if self.mag['y'] > magymax:
            magymax = self.mag['y']
        if self.mag['y'] < magymin:
            magymin = self.mag['y']
        
        x = self.mag['x'] - 43 #ズレ分を引く
        y = self.mag['y'] - 15

        #北 -1, 南 +1   西 -1 東 +1 
        xd = (x ** 2 / (x**2 + y**2)) ** 0.5 #方向成分を計算
        if x < 0:
            xd *= -1 #負の値を計算
        yd = (y ** 2 / (x**2 + y**2)) ** 0.5
        if y < 0:
            yd *= -1 #負の値を計算  
        
        return xd, yd, self.temp

    def shutdown(self):
        global magxmax
        global magxmin
        global magymax
        global magymin

        print("IMU Mag X MAX %5.1f MIN %5.1f  Y MAX %5.1f MIN %5.1f" % (magxmax,magxmin,magymax,magymin))
        self.on = False


if __name__ == "__main__":
    iter = 0
    import sys
    sensor_type = SENSOR_MPU9250 
    dlp_setting = DLP_SETTING_DISABLED
    if len(sys.argv) > 1:
        sensor_type = sys.argv[1]
    if len(sys.argv) > 2:
        dlp_setting = int(sys.argv[2])

    p = IMU(sensor=sensor_type)
    while iter < 100:
        data = p.run()
        #print(data)
        time.sleep(0.1)
        iter += 1
     
