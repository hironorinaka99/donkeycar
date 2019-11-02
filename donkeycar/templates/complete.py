#!/usr/bin/env python3

"""
Scripts to drive a donkey 2 car

Usage:
    manage.py (drive) [--model=<model>] [--js] [--type=(linear|categorical|rnn|imu|behavior|3d|localizer|latent)] [--camera=(single|stereo)] [--meta=<key:value> ...]
    manage.py (train) [--tub=<tub1,tub2,..tubn>] [--file=<file> ...] (--model=<model>) [--transfer=<model>] [--type=(linear|categorical|rnn|imu|behavior|3d|localizer)] [--continuous] [--aug]


Options:
    -h --help          Show this screen.
    --js               Use physical joystick.
    -f --file=<file>   A text file containing paths to tub files, one per line. Option may be used more than once.
    --meta=<key:value> Key/Value strings describing describing a piece of meta data about this drive. Option may be used more than once.
"""
import os
import time
from docopt import docopt
import numpy as np

import donkeycar as dk

#import parts
from donkeycar.parts.transform import Lambda, TriggeredCallback, DelayedTrigger
from donkeycar.parts.datastore import TubHandler
from donkeycar.parts.controller import LocalWebController, JoystickController
from donkeycar.parts.throttle_filter import ThrottleFilter
from donkeycar.parts.behavior import BehaviorPart
from donkeycar.parts.file_watcher import FileWatcher
from donkeycar.parts.launch import AiLaunch
from donkeycar.utils import *

time_dis_short_start = 0 ##バック入力の為のダミー初期時刻

"""
global prev_distanceLL
global prev_distanceL
global prev_distanceC
global prev_distanceR
global prev_distanceRR
"""

def drive(cfg, model_path=None, use_joystick=False, model_type=None, camera_type='single', meta=[] ):
    '''
    Construct a working robotic vehicle from many parts.
    Each part runs as a job in the Vehicle loop, calling either
    it's run or run_threaded method depending on the constructor flag `threaded`.
    All parts are updated one after another at the framerate given in
    cfg.DRIVE_LOOP_HZ assuming each part finishes processing in a timely manner.
    Parts may have named outputs and inputs. The framework handles passing named outputs
    to parts requesting the same named input.
    '''

    if cfg.DONKEY_GYM:
        #the simulator will use cuda and then we usually run out of resources
        #if we also try to use cuda. so disable for donkey_gym.
        os.environ["CUDA_VISIBLE_DEVICES"]="-1" 

    if model_type is None:
        if cfg.TRAIN_LOCALIZER:
            model_type = "localizer"
        elif cfg.TRAIN_BEHAVIORS:
            model_type = "behavior"
        else:
            model_type = cfg.DEFAULT_MODEL_TYPE
    
    #Initialize car
    V = dk.vehicle.Vehicle()

    if camera_type == "stereo":

        if cfg.CAMERA_TYPE == "WEBCAM":
            from donkeycar.parts.camera import Webcam            

            camA = Webcam(image_w=cfg.IMAGE_W, image_h=cfg.IMAGE_H, image_d=cfg.IMAGE_DEPTH, iCam = 0)
            camB = Webcam(image_w=cfg.IMAGE_W, image_h=cfg.IMAGE_H, image_d=cfg.IMAGE_DEPTH, iCam = 1)

        elif cfg.CAMERA_TYPE == "CVCAM":
            from donkeycar.parts.cv import CvCam

            camA = CvCam(image_w=cfg.IMAGE_W, image_h=cfg.IMAGE_H, image_d=cfg.IMAGE_DEPTH, iCam = 0)
            camB = CvCam(image_w=cfg.IMAGE_W, image_h=cfg.IMAGE_H, image_d=cfg.IMAGE_DEPTH, iCam = 1)
        else:
            raise(Exception("Unsupported camera type: %s" % cfg.CAMERA_TYPE))

        V.add(camA, outputs=['cam/image_array_a'], threaded=True)
        V.add(camB, outputs=['cam/image_array_b'], threaded=True)

        from donkeycar.parts.image import StereoPair

        V.add(StereoPair(), inputs=['cam/image_array_a', 'cam/image_array_b'], 
            outputs=['cam/image_array'])

    else:
        print("cfg.CAMERA_TYPE", cfg.CAMERA_TYPE)
        if cfg.DONKEY_GYM:
            from donkeycar.parts.dgym import DonkeyGymEnv 
        
        inputs = []
        threaded = True
        print("cfg.CAMERA_TYPE", cfg.CAMERA_TYPE)
        if cfg.DONKEY_GYM:
            from donkeycar.parts.dgym import DonkeyGymEnv 
            cam = DonkeyGymEnv(cfg.DONKEY_SIM_PATH, env_name=cfg.DONKEY_GYM_ENV_NAME)
            threaded = True
            inputs = ['angle', 'throttle']
        elif cfg.CAMERA_TYPE == "PICAM":
            from donkeycar.parts.camera import PiCamera
            cam = PiCamera(image_w=cfg.IMAGE_W, image_h=cfg.IMAGE_H, image_d=cfg.IMAGE_DEPTH)
        elif cfg.CAMERA_TYPE == "WEBCAM":
            from donkeycar.parts.camera import Webcam
            cam = Webcam(image_w=cfg.IMAGE_W, image_h=cfg.IMAGE_H, image_d=cfg.IMAGE_DEPTH)
        elif cfg.CAMERA_TYPE == "CVCAM":
            from donkeycar.parts.cv import CvCam
            cam = CvCam(image_w=cfg.IMAGE_W, image_h=cfg.IMAGE_H, image_d=cfg.IMAGE_DEPTH)
        elif cfg.CAMERA_TYPE == "CSIC":
            from donkeycar.parts.camera import CSICamera
            cam = CSICamera(image_w=cfg.IMAGE_W, image_h=cfg.IMAGE_H, image_d=cfg.IMAGE_DEPTH, framerate=cfg.CAMERA_FRAMERATE, gstreamer_flip=cfg.CSIC_CAM_GSTREAMER_FLIP_PARM)
        elif cfg.CAMERA_TYPE == "V4L":
            from donkeycar.parts.camera import V4LCamera
            cam = V4LCamera(image_w=cfg.IMAGE_W, image_h=cfg.IMAGE_H, image_d=cfg.IMAGE_DEPTH, framerate=cfg.CAMERA_FRAMERATE)
        elif cfg.CAMERA_TYPE == "MOCK":
            from donkeycar.parts.camera import MockCamera
            cam = MockCamera(image_w=cfg.IMAGE_W, image_h=cfg.IMAGE_H, image_d=cfg.IMAGE_DEPTH)
        else:
            raise(Exception("Unkown camera type: %s" % cfg.CAMERA_TYPE))
            
        V.add(cam, inputs=inputs, outputs=['cam/image_array'], threaded=threaded)
        
    if use_joystick or cfg.USE_JOYSTICK_AS_DEFAULT:
        #modify max_throttle closer to 1.0 to have more power
        #modify steering_scale lower than 1.0 to have less responsive steering
        from donkeycar.parts.controller import get_js_controller
        
        ctr = get_js_controller(cfg)
        
        if cfg.USE_NETWORKED_JS:
            from donkeycar.parts.controller import JoyStickSub
            netwkJs = JoyStickSub(cfg.NETWORK_JS_SERVER_IP)
            V.add(netwkJs, threaded=True)
            ctr.js = netwkJs

    else:        
        #This web controller will create a web server that is capable
        #of managing steering, throttle, and modes, and more.
        ctr = LocalWebController()

    
    V.add(ctr, 
          inputs=['cam/image_array'],
          outputs=['user/angle', 'user/throttle', 'user/mode', 'recording'],
          threaded=True)

    #this throttle filter will allow one tap back for esc reverse
    th_filter = ThrottleFilter()
    V.add(th_filter, inputs=['user/throttle'], outputs=['user/throttle'])
    
    #See if we should even run the pilot module. 
    #This is only needed because the part run_condition only accepts boolean
    class PilotCondition:
        def run(self, mode):
            if mode == 'user':
                return False
            else:
                return True       

    V.add(PilotCondition(), inputs=['user/mode'], outputs=['run_pilot'])
    
    # Distance sensor part  Nakagawa-add
    from donkeycar.parts.DistanceSensorMulti4 import DistanceSensorMulti4
    distanceSensorMultiPart4 = DistanceSensorMulti4()
    V.add(distanceSensorMultiPart4,
        outputs=['distanceLL','distanceL','distanceC','distanceR','distanceRR','prev_distanceLL', 'prev_distanceL', 'prev_distanceC', 'prev_distanceR','prev_distanceRR'],
        #run_condition='user',
        #run_condition='run_pilot',
        threaded=True)

    class LedConditionLogic:
        def __init__(self, cfg):
            self.cfg = cfg

        def run(self, mode, recording, recording_alert, behavior_state, model_file_changed, track_loc):
            #returns a blink rate. 0 for off. -1 for on. positive for rate.
            
            if track_loc is not None:
                led.set_rgb(*self.cfg.LOC_COLORS[track_loc])
                return -1

            if model_file_changed:
                led.set_rgb(self.cfg.MODEL_RELOADED_LED_R, self.cfg.MODEL_RELOADED_LED_G, self.cfg.MODEL_RELOADED_LED_B)
                return 0.1
            else:
                led.set_rgb(self.cfg.LED_R, self.cfg.LED_G, self.cfg.LED_B)

            if recording_alert:
                led.set_rgb(*recording_alert)
                return self.cfg.REC_COUNT_ALERT_BLINK_RATE
            else:
                led.set_rgb(self.cfg.LED_R, self.cfg.LED_G, self.cfg.LED_B)
        
            if behavior_state is not None and model_type == 'behavior':
                r, g, b = self.cfg.BEHAVIOR_LED_COLORS[behavior_state]
                led.set_rgb(r, g, b)
                return -1 #solid on

            if recording:
                return -1 #solid on
            elif mode == 'user':
                return 1
            elif mode == 'local_angle':
                return 0.5
            elif mode == 'local':
                return 0.1
            return 0

    if cfg.HAVE_RGB_LED and not cfg.DONKEY_GYM:
        from donkeycar.parts.led_status import RGB_LED
        led = RGB_LED(cfg.LED_PIN_R, cfg.LED_PIN_G, cfg.LED_PIN_B, cfg.LED_INVERT)
        led.set_rgb(cfg.LED_R, cfg.LED_G, cfg.LED_B)        
        
        V.add(LedConditionLogic(cfg), inputs=['user/mode', 'recording', "records/alert", 'behavior/state', 'modelfile/modified', "pilot/loc"],
              outputs=['led/blink_rate'])

        V.add(led, inputs=['led/blink_rate'])
        

    def get_record_alert_color(num_records):
        col = (0, 0, 0)
        for count, color in cfg.RECORD_ALERT_COLOR_ARR:
            if num_records >= count:
                col = color
        return col    

    class RecordTracker:
        def __init__(self):
            self.last_num_rec_print = 0
            self.dur_alert = 0
            self.force_alert = 0

        def run(self, num_records):
            if num_records is None:
                return 0
            
            if self.last_num_rec_print != num_records or self.force_alert:
                self.last_num_rec_print = num_records

                if num_records % 10 == 0:
                    print("recorded", num_records, "records")
                        
                if num_records % cfg.REC_COUNT_ALERT == 0 or self.force_alert:
                    self.dur_alert = num_records // cfg.REC_COUNT_ALERT * cfg.REC_COUNT_ALERT_CYC
                    self.force_alert = 0
                    
            if self.dur_alert > 0:
                self.dur_alert -= 1

            if self.dur_alert != 0:
                return get_record_alert_color(num_records)

            return 0

    rec_tracker_part = RecordTracker()
    V.add(rec_tracker_part, inputs=["tub/num_records"], outputs=['records/alert'])

    if cfg.AUTO_RECORD_ON_THROTTLE and isinstance(ctr, JoystickController):
        #then we are not using the circle button. hijack that to force a record count indication
        def show_record_acount_status():
            rec_tracker_part.last_num_rec_print = 0
            rec_tracker_part.force_alert = 1
        ctr.set_button_down_trigger('circle', show_record_acount_status)

    #Sombrero
    if cfg.HAVE_SOMBRERO:
        from donkeycar.parts.sombrero import Sombrero
        s = Sombrero()

    #IMU
    if cfg.HAVE_IMU:
        from donkeycar.parts.imu import Mpu6050
        imu = Mpu6050()
        V.add(imu, outputs=['imu/acl_x', 'imu/acl_y', 'imu/acl_z',
            'imu/gyr_x', 'imu/gyr_y', 'imu/gyr_z'], threaded=True)

    class ImgPreProcess():
        '''
        preprocess camera image for inference.
        normalize and crop if needed.
        '''
        def __init__(self, cfg):
            self.cfg = cfg

        def run(self, img_arr):
            return normalize_and_crop(img_arr, self.cfg)

    if "coral" in model_type:
        inf_input = 'cam/image_array'
    else:
        inf_input = 'cam/normalized/cropped'
        V.add(ImgPreProcess(cfg),
            inputs=['cam/image_array'],
            outputs=[inf_input],
            run_condition='run_pilot')

    #Behavioral state
    if cfg.TRAIN_BEHAVIORS:
        bh = BehaviorPart(cfg.BEHAVIOR_LIST)
        V.add(bh, outputs=['behavior/state', 'behavior/label', "behavior/one_hot_state_array"])
        try:
            ctr.set_button_down_trigger('L1', bh.increment_state)
        except:
            pass

        inputs = [inf_input, "behavior/one_hot_state_array"]  
    #IMU
    elif model_type == "imu":
        assert(cfg.HAVE_IMU)
        #Run the pilot if the mode is not user.
        inputs=[inf_input,
            'imu/acl_x', 'imu/acl_y', 'imu/acl_z',
            'imu/gyr_x', 'imu/gyr_y', 'imu/gyr_z']
    else:
        inputs=[inf_input]

    def load_model(kl, model_path):
        start = time.time()
        print('loading model', model_path)
        kl.load(model_path)
        print('finished loading in %s sec.' % (str(time.time() - start)) )

    def load_weights(kl, weights_path):
        start = time.time()
        try:
            print('loading model weights', weights_path)
            kl.model.load_weights(weights_path)
            print('finished loading in %s sec.' % (str(time.time() - start)) )
        except Exception as e:
            print(e)
            print('ERR>> problems loading weights', weights_path)

    def load_model_json(kl, json_fnm):
        start = time.time()
        print('loading model json', json_fnm)
        from tensorflow.python import keras
        try:
            with open(json_fnm, 'r') as handle:
                contents = handle.read()
                kl.model = keras.models.model_from_json(contents)
            print('finished loading json in %s sec.' % (str(time.time() - start)) )
        except Exception as e:
            print(e)
            print("ERR>> problems loading model json", json_fnm)

    if model_path:
        #When we have a model, first create an appropriate Keras part
        kl = dk.utils.get_model_by_type(model_type, cfg)

        model_reload_cb = None

        if '.h5' in model_path or '.uff' in model_path or 'tflite' in model_path or '.pkl' in model_path:
            #when we have a .h5 extension
            #load everything from the model file
            load_model(kl, model_path)

            def reload_model(filename):
                load_model(kl, filename)

            model_reload_cb = reload_model

        elif '.json' in model_path:
            #when we have a .json extension
            #load the model from there and look for a matching
            #.wts file with just weights
            load_model_json(kl, model_path)
            weights_path = model_path.replace('.json', '.weights')
            load_weights(kl, weights_path)

            def reload_weights(filename):
                weights_path = filename.replace('.json', '.weights')
                load_weights(kl, weights_path)
            
            model_reload_cb = reload_weights

        else:
            print("ERR>> Unknown extension type on model file!!")
            return

        #this part will signal visual LED, if connected
        V.add(FileWatcher(model_path, verbose=True), outputs=['modelfile/modified'])

        #these parts will reload the model file, but only when ai is running so we don't interrupt user driving
        V.add(FileWatcher(model_path), outputs=['modelfile/dirty'], run_condition="ai_running")
        V.add(DelayedTrigger(100), inputs=['modelfile/dirty'], outputs=['modelfile/reload'], run_condition="ai_running")
        V.add(TriggeredCallback(model_path, model_reload_cb), inputs=["modelfile/reload"], run_condition="ai_running")

        outputs=['pilot/angle', 'pilot/throttle']

        if cfg.TRAIN_LOCALIZER:
            outputs.append("pilot/loc")
    
        V.add(kl, inputs=inputs, 
            outputs=outputs,
            run_condition='run_pilot')            

    #Choose what inputs should change the car.
    class DriveMode:
        def run(self, mode, 
                    user_angle, user_throttle,
                    pilot_angle, pilot_throttle,distanceLL,distanceL,distanceC,distanceR,distanceRR,prev_distanceLL,prev_distanceL,prev_distanceC,prev_distanceR,prev_distanceRR):

            #print("Drive Mode:" + mode)
            global time_dis_short_start
            dis_LL_range = 20 #左横センサーの反応範囲
            dis_LL_rev_range = 8 #左横センサーの後退反応範囲
            dis_L_range = 20 #左センサーの反応範囲 
            dis_C_range = 25 #中央センサーの反応範囲 
            dis_R_range = 20 #右センサーの反応範囲 
            dis_RR_range = 20 #右横センサーの反応範囲 
            dis_RR_rev_range = 8 #右横センサーの後退反応範囲
            dis_LLRR_value = 0.03 #横センサーの反応係数

            dis_timer_all = 0.5 #待ち時間全体 下記2つの時間より長いこと
            dis_timer_back = 0.4 #後退時間
            dis_timer_wait = 0.05 #後退待ち時間
            #dis_back_throttle = -0.35 #後退速度
            dis_back_throttle = -1.1 * abs(user_throttle) #おおよそ 0.3-0.35

            angle_adj_1 = 0.5 #惰性前進時のハンドル修正 #初回完走時0.5
            angle_adj_2 = 0.2 #中央センサが近い時、開けている方向に向くハンドル操作値

            dis_L_LKA_range = 10.0 #左センサーLKA動作範囲
            dis_R_LKA_range = 10.0 #右センサーLKA動作範囲
            dis_LR_value = 0.01 #左右センサーLKA反応係数

            #前回測定時との比較　近づいている時は負、離れているときは正、値が近いときはばらつき誤差として０とする
            dis_gap_ignor_range_side = 0.5 #（横）センサーばらつきで、前回差を０とする範囲
            dis_gap_ignor_range_front = 1.0 #（前）センサーばらつきで、前回差を０とする範囲 1未満は0とする
            dis_gap_ignor_range_fast = 10

            dis_gapLL = distanceLL - prev_distanceLL
            if abs(dis_gapLL) < dis_gap_ignor_range_side or abs(dis_gapLL) > dis_gap_ignor_range_fast: dis_gapLL = 0 
            dis_gapL = distanceL - prev_distanceL
            if abs(dis_gapL) < dis_gap_ignor_range_front or abs(dis_gapL) > dis_gap_ignor_range_fast: dis_gapL = 0 
            dis_gapC = distanceC - prev_distanceC
            if abs(dis_gapC) < dis_gap_ignor_range_front or abs(dis_gapC) > dis_gap_ignor_range_fast: dis_gapC = 0 
            dis_gapR = distanceR - prev_distanceR
            if abs(dis_gapR) < dis_gap_ignor_range_front or abs(dis_gapR) > dis_gap_ignor_range_fast: dis_gapR = 0 
            dis_gapRR = distanceRR - prev_distanceRR
            if abs(dis_gapRR) < dis_gap_ignor_range_side or abs(dis_gapRR) > dis_gap_ignor_range_fast: dis_gapRR = 0 


            if mode == 'user': 

                #ステアリング狙い値出し（左右に振る）
                t = int(time.time()*10)%2 #0.05秒単位
                if abs(user_angle) < 0.2:
                    if t == 0:
                        user_angle += 0.2
                    else:
                        user_angle -= 0.2

                """
                #条件が良い時には加速
                #print ("LL: %.1f cm" % distanceLL +"L: %.1f cm" % distanceL +"  " "C: %.1f cm" % distanceC + "  " "R: %.1f cm" % distanceR + "  " "RR: %.1f cm" % distanceRR) 
                #print("front left gap %3.1f cm" % dis_gapL + "front cencer gap %3.1f cm" % dis_gapC + "front right gap %3.1f cm" % dis_gapR)
                if distanceLL > 12 and distanceL > 50 and distanceC > 80 and distanceR > 50 and distanceRR > 12: #順全開条件
                    #print("準全開条件成立")
                    if distanceL > 60 and distanceC > 100 and distanceR > 60: #全開条件
                        #print("全開条件成立")
                        if dis_gapL >= 0 and dis_gapC >= 0 and dis_gapR >=0: #前のセンサー距離がどれも縮まっていない
                            #print("ギャップ条件成立")
                            user_angle *= 1.2 #全開条件整ったら
                            user_throttle *= 1.2
                            #print("boost 1.2")
                        #else:
                            #print("距離が縮まっているため全開ブーストなし")              

                    else: #準全開条件
                        if dis_gapL >= 0 and dis_gapC >= 0 and dis_gapR >=0: #前のセンサー距離がどれも縮まっていない
                            user_angle *= 1.1 #準全開条件整ったら
                            user_throttle *= 1.1
                            #print("boost 1.1")
                        #else:
                            #print("距離が縮まっているため準全開ブーストなし")              

                #距離センサーのギャップ（縮まり方）が大きいときは大減速
                elif (distanceL < 80 and distanceL > 20 and dis_gapL < -2.0 and user_angle < -0.3) or (distanceC < 120 and distanceC > 25 and dis_gapC < -2.0 and abs(user_angle) < 0.5) or (distanceR < 80 and distanceR > 20 and dis_gapR < -2.0 and user_angle > 0.3): #前センサーで障害物（距離センサーが縮まっている）発見
                    print("front left gap %3.1f cm" % dis_gapL + "front cencer gap %3.1f cm" % dis_gapC + "front right gap %3.1f cm" % dis_gapR)
                    user_throttle *= 0.0
                    print("急速接近中のため　スロットル０")
                
                #距離センサーのギャップ（縮まり方）がそこまで大きくないときは小減速
                elif (distanceL < 80 and distanceL > 20 and dis_gapL < -1.0 and user_angle < -0.3) or (distanceC < 150 and distanceC > 25 and dis_gapC < -2.0 and abs(user_angle) < 0.5) or (distanceR < 80 and distanceR > 20 and dis_gapR < -1.0 and user_angle > 0.3): #前センサーで障害物（距離センサーが縮まっている）発見
                    print("front left gap %3.1f cm" % dis_gapL + "front cencer gap %3.1f cm" % dis_gapC + "front right gap %3.1f cm" % dis_gapR)
                    user_throttle *= 0.5 #テストで０
                    print("微速接近中のため、　スロットル0.5")
                                       
                #LKA的な動作    真横　#ハンドル右はプラス、左はマイナス 離れていっているとき(gapが正)は行わない
                if distanceLL < dis_LL_range and distanceLL > 0: #左横センサ近いとき (マイナス値、離れていっているときは除く)
                    if dis_gapLL < 0: #gapが減っているときのみ補正
                        user_angle += 0.20 + (dis_LL_range - distanceLL) * dis_LLRR_value  #ハンドル指示値を右に少し 0.2+係数分
                    else:
                        print("離れ始めたので補正しない")
                if distanceRR < dis_RR_range and distanceRR > 0: #右横センサ近いとき(マイナス値、、離れていっているときは除く)
                    if dis_gapRR < 0: #gapが減っているときのみ補正
                        user_angle -= 0.20 + (dis_RR_range - distanceRR) * dis_LLRR_value  #ハンドル指示値を右に少し 0.2+係数分               
                    else:
                        print("離れ始めたので補正しない")

                
                #LKA的な動作　左右前センサー分
                if distanceL - dis_L_range < dis_L_LKA_range and distanceL - dis_L_range >0: #左センサーが反応範囲に近いとき（マイナス値は除く）
                    user_angle += 0.1 + (dis_L_LKA_range - (distanceL - dis_L_range)) * dis_LR_value #初期値　0.2 +LKA_Rangeの残り分ｘ係数
                if distanceR - dis_R_range < dis_R_LKA_range and distanceR - dis_R_range >0: #右センサーが反応範囲に近いとき（マイナス値は除く）
                    user_angle -= 0.1 + (dis_R_LKA_range - (distanceR - dis_R_range)) * dis_LR_value #初期値　0.2 +LKA_Rangeの残り分ｘ係数　           
                

                #後退させる必要があるとき
                if distanceLL < dis_LL_rev_range or distanceL < dis_L_range or distanceC < dis_C_range or distanceR < dis_R_range or distanceRR < dis_RR_rev_range :
                    time_dis_gap = time.time() - time_dis_short_start
                    if time_dis_gap > dis_timer_all: #初期タイマー無反応（下記数値より大きいこと）
                        time_dis_short_start = time.time()
                        #print("set new start time")
                        return user_angle, 0 #ニュートラルに戻す

                    elif time_dis_gap < dis_timer_wait: #バックする為に一度0を入力
                        #print("back wait" + str(time_dis_gap))
                        print("back wait %3.1f" % time_dis_gap)
                        if distanceLL < dis_LL_rev_range or min(distanceL, distanceC, distanceR) == distanceL and distanceLL > dis_LL_rev_range and distanceRR > dis_RR_rev_range: #左前が近く、横センサーが反応していない条件:
                            return user_angle + angle_adj_1, 0 #左が近い場合は少し(angle_adj_1分)右に切って(惰性前進中)、スロットル0で待機
                        elif distanceRR < dis_RR_rev_range or min(distanceL, distanceC, distanceR) == distanceR and distanceLL > dis_LL_rev_range and distanceRR > dis_RR_rev_range: #右前が近く、横センサーが反応していない条件:
                            return user_angle - angle_adj_1, 0 #右が近い場合は少し(angle_adj_1分)左に切って(惰性前進中)、スロットル0で待機
                        else:
                            return user_angle, 0 #中央が近い場合は、スロットル0で待機

                    elif time_dis_gap < dis_timer_wait + 0.1: #バック開始の短い時間は、userAngleで後退
                        print("back keep angle")
                        if min(distanceL, distanceC, distanceR) == distanceL and distanceLL > dis_LL_rev_range and distanceRR > dis_RR_rev_range: #左前が近く、横センサーが反応していない条件:
                            return user_angle, dis_back_throttle #左が近い場合は切り増しをやめて、短時間後退
                        elif min(distanceL, distanceC, distanceR) == distanceR and distanceLL > dis_LL_rev_range and distanceRR > dis_RR_rev_range: #右前が近く、横センサーが反応していない条件:
                            return user_angle, dis_back_throttle #左が近い場合は切り増しをやめて、短時間後退
                        else:
                            return 0, dis_back_throttle #中央が近い場合は、ステアリング中立で後退

                    elif time_dis_gap < (dis_timer_back + dis_timer_wait): #いったんバックする時間
                        #print("back" + str(time_dis_gap))
                        print("back %3.1f" % time_dis_gap) 
                        if max(distanceL, distanceC * 1.5, distanceR) == distanceC *1.5 and distanceL < dis_L_range *1.5 and distanceC < dis_C_range *1.5 and distanceR < dis_R_range *1.5 and distanceLL > dis_LL_rev_range and distanceRR > dis_RR_rev_range: #左右前センサより中央が遠く、前のセンサー3つがどれも近く(係数倍）、横センサーが反応していない条件（角に向いてしまった場合）
                            print("角向きのため、右向くために下がる")
                            return -1, dis_back_throttle #左にハンドル切って後退　（できるだけ右回りに戻りたいので）
                        elif min(distanceL, distanceC, distanceR) == distanceL and distanceLL > dis_LL_rev_range and distanceRR > dis_RR_rev_range: #左前が近く、横センサーが反応していない条件
                            return -1, dis_back_throttle #左が近い場合は、左にハンドル切って後退
                        elif min(distanceL, distanceC, distanceR) == distanceR and distanceLL > dis_LL_rev_range and distanceRR > dis_RR_rev_range: #右前が近く、横センサーが反応していない条件
                            return 1, dis_back_throttle #右が近い場合は、右にハンドル切って後退
                        else:                           #中央が近い場合
                            if distanceL > distanceR:   #中央が近く、左側が大きく開いている場合、ハンドルを少し右(angle_adj_2 * -1.0)に切って後退
                                print("中央センサ停止、左センサ方向空きの為、右に切って下がる")
                                return angle_adj_2 * -1.0 , dis_back_throttle 
                            else:                       #中央が近く、左側が大きく開いている場合、ハンドルを少し右(angle_adj_2 * 1.0)に切って後退
                                print("中央センサ停止、右センサ方向空きの為、左に切って下がる")
                                return angle_adj_2 * 1.0, dis_back_throttle
                else:
                    return user_angle, user_throttle * cfg.AI_THROTTLE_MULT #使える？


                """
                return user_angle, user_throttle
                                
            elif mode == 'local_angle':
                #ステアリング狙い値出し（左右に振る）
                t = int(time.time()*10)%2 #0.05秒単位
                if abs(pilot_angle) < 0.2:
                    if t == 0:
                        pilot_angle += 0.2
                    else:
                        pilot_angle -= 0.2

                #条件が良い時には加速
                #print ("LL: %.1f cm" % distanceLL +"L: %.1f cm" % distanceL +"  " "C: %.1f cm" % distanceC + "  " "R: %.1f cm" % distanceR + "  " "RR: %.1f cm" % distanceRR) 
                #print("front left gap %3.1f cm" % dis_gapL + "front cencer gap %3.1f cm" % dis_gapC + "front right gap %3.1f cm" % dis_gapR)
                if distanceLL > 12 and distanceL > 50 and distanceC > 80 and distanceR > 50 and distanceRR > 12: #順全開条件
                    #print("準全開条件成立")
                    if distanceL > 60 and distanceC > 100 and distanceR > 60: #全開条件
                        #print("全開条件成立")
                        if dis_gapL >= 0 and dis_gapC >= 0 and dis_gapR >=0: #前のセンサー距離がどれも縮まっていない
                            #print("ギャップ条件成立")
                            pilot_angle *= 1.1 #全開条件整ったら
                            user_throttle *= 1.1
                            #print("boost 1.1")
                        #else:
                            #print("距離が縮まっているため全開ブーストなし")              

                    else: #準全開条件
                        if dis_gapL >= 0 and dis_gapC >= 0 and dis_gapR >=0: #前のセンサー距離がどれも縮まっていない
                            pilot_angle *= 1.05 #準全開条件整ったら
                            user_throttle *= 1.05
                            #print("boost 1.05")
                        #else:
                            #print("距離が縮まっているため準全開ブーストなし")              

                #中距離で距離センサーのギャップ（縮まり方）が大きいときは大減速
                elif (distanceL < 60 and distanceL > 20 and dis_gapL < -2.0 and pilot_angle < -0.3) or (distanceC < 100 and distanceC > 25 and dis_gapC < -2.5 and abs(pilot_angle) < 0.5) or (distanceR < 60 and distanceR > 20 and dis_gapR < -2.0 and pilot_angle > 0.3): #前センサーで障害物（距離センサーが縮まっている）発見
                    print("front left gap %3.1f cm" % dis_gapL + "front cencer gap %3.1f cm" % dis_gapC + "front right gap %3.1f cm" % dis_gapR)
                    user_throttle *= 0.0
                    print("急速接近中のため　スロットル０")

                #遠くで距離センサーのギャップ（縮まり方）が大きいときは中減速
                elif (distanceL < 80 and distanceL > 60 and dis_gapL < -2.0 and pilot_angle < -0.3) or (distanceC < 140 and distanceC > 100 and dis_gapC < -3.0 and abs(pilot_angle) < 0.5) or (distanceR < 80 and distanceR > 60 and dis_gapR < -2.0 and pilot_angle > 0.3): #前センサーで障害物（距離センサーが縮まっている）発見
                    print("front left gap %3.1f cm" % dis_gapL + "front cencer gap %3.1f cm" % dis_gapC + "front right gap %3.1f cm" % dis_gapR)
                    user_throttle *= 0.1
                    print("急速接近中のため　スロットル０．２")
                
                

                #距離センサーのギャップ（縮まり方）がそこまで大きくないときは小減速
                elif (distanceL < 80 and distanceL > 20 and dis_gapL < -1.0 and pilot_angle < -0.3) or (distanceC < 150 and distanceC > 25 and dis_gapC < -2.0 and abs(pilot_angle) < 0.5) or (distanceR < 80 and distanceR > 20 and dis_gapR < -1.0 and pilot_angle > 0.3): #前センサーで障害物（距離センサーが縮まっている）発見
                    print("front left gap %3.1f cm" % dis_gapL + "front cencer gap %3.1f cm" % dis_gapC + "front right gap %3.1f cm" % dis_gapR)
                    user_throttle *= 0.5 #テストで０
                    print("微速接近中のため、　スロットル0.5")
                                       
                #LKA的な動作    真横　#ハンドル右はプラス、左はマイナス 離れていっているとき(gapが正)は行わない
                if distanceLL < dis_LL_range and distanceLL > 0: #左横センサ近いとき (マイナス値、離れていっているときは除く)
                    if dis_gapLL < 0: #gapが減っているときのみ補正
                        pilot_angle += 0.20 + (dis_LL_range - distanceLL) * dis_LLRR_value  #ハンドル指示値を右に少し 0.2+係数分
                    else:
                        print("離れ始めたので補正しない")
                if distanceRR < dis_RR_range and distanceRR > 0: #右横センサ近いとき(マイナス値、、離れていっているときは除く)
                    if dis_gapRR < 0: #gapが減っているときのみ補正
                        pilot_angle -= 0.20 + (dis_RR_range - distanceRR) * dis_LLRR_value  #ハンドル指示値を右に少し 0.2+係数分               
                    else:
                        print("離れ始めたので補正しない")

                
                #LKA的な動作　左右前センサー分
                if distanceL - dis_L_range < dis_L_LKA_range and distanceL - dis_L_range >0: #左センサーが反応範囲に近いとき（マイナス値は除く）
                    pilot_angle += 0.1 + (dis_L_LKA_range - (distanceL - dis_L_range)) * dis_LR_value #初期値　0.2 +LKA_Rangeの残り分ｘ係数
                if distanceR - dis_R_range < dis_R_LKA_range and distanceR - dis_R_range >0: #右センサーが反応範囲に近いとき（マイナス値は除く）
                    pilot_angle -= 0.1 + (dis_R_LKA_range - (distanceR - dis_R_range)) * dis_LR_value #初期値　0.2 +LKA_Rangeの残り分ｘ係数　           
                

                #後退させる必要があるとき
                if distanceLL < dis_LL_rev_range or distanceL < dis_L_range or distanceC < dis_C_range or distanceR < dis_R_range or distanceRR < dis_RR_rev_range :
                    time_dis_gap = time.time() - time_dis_short_start
                    if time_dis_gap > dis_timer_all: #初期タイマー無反応（下記数値より大きいこと）
                        time_dis_short_start = time.time()
                        #print("set new start time")
                        return pilot_angle, 0 #ニュートラルに戻す

                    elif time_dis_gap < dis_timer_wait: #バックする為に一度0を入力
                        #print("back wait" + str(time_dis_gap))
                        print("back wait %3.1f" % time_dis_gap)
                        if distanceLL < dis_LL_rev_range or min(distanceL, distanceC, distanceR) == distanceL and distanceLL > dis_LL_rev_range and distanceRR > dis_RR_rev_range: #左前が近く、横センサーが反応していない条件:
                            return pilot_angle + angle_adj_1, 0 #左が近い場合は少し(angle_adj_1分)右に切って(惰性前進中)、スロットル0で待機
                        elif distanceRR < dis_RR_rev_range or min(distanceL, distanceC, distanceR) == distanceR and distanceLL > dis_LL_rev_range and distanceRR > dis_RR_rev_range: #右前が近く、横センサーが反応していない条件:
                            return pilot_angle - angle_adj_1, 0 #右が近い場合は少し(angle_adj_1分)左に切って(惰性前進中)、スロットル0で待機
                        else:
                            return pilot_angle, 0 #中央が近い場合は、スロットル0で待機

                    elif time_dis_gap < dis_timer_wait + 0.1: #バック開始の短い時間は、userAngleで後退
                        print("back keep angle")
                        if min(distanceL, distanceC, distanceR) == distanceL and distanceLL > dis_LL_rev_range and distanceRR > dis_RR_rev_range: #左前が近く、横センサーが反応していない条件:
                            return pilot_angle, dis_back_throttle #左が近い場合は切り増しをやめて、短時間後退
                        elif min(distanceL, distanceC, distanceR) == distanceR and distanceLL > dis_LL_rev_range and distanceRR > dis_RR_rev_range: #右前が近く、横センサーが反応していない条件:
                            return pilot_angle, dis_back_throttle #左が近い場合は切り増しをやめて、短時間後退
                        else:
                            return 0, dis_back_throttle #中央が近い場合は、ステアリング中立で後退

                    elif time_dis_gap < (dis_timer_back + dis_timer_wait): #いったんバックする時間
                        #print("back" + str(time_dis_gap))
                        print("back %3.1f" % time_dis_gap) 
                        if max(distanceL, distanceC * 1.5, distanceR) == distanceC *1.5 and distanceL < dis_L_range *1.5 and distanceC < dis_C_range *1.5 and distanceR < dis_R_range *1.5 and distanceLL > dis_LL_rev_range and distanceRR > dis_RR_rev_range: #左右前センサより中央が遠く、前のセンサー3つがどれも近く(係数倍）、横センサーが反応していない条件（角に向いてしまった場合）
                            print("角向きのため、右向くために下がる")
                            return -1, dis_back_throttle #左にハンドル切って後退　（できるだけ右回りに戻りたいので）
                        elif min(distanceL, distanceC, distanceR) == distanceL and distanceLL > dis_LL_rev_range and distanceRR > dis_RR_rev_range: #左前が近く、横センサーが反応していない条件
                            return -1, dis_back_throttle #左が近い場合は、左にハンドル切って後退
                        elif min(distanceL, distanceC, distanceR) == distanceR and distanceLL > dis_LL_rev_range and distanceRR > dis_RR_rev_range: #右前が近く、横センサーが反応していない条件
                            return 1, dis_back_throttle #右が近い場合は、右にハンドル切って後退
                        else:                           #中央が近い場合
                            if distanceL > distanceR:   #中央が近く、左側が大きく開いている場合、ハンドルを少し右(angle_adj_2 * -1.0)に切って後退
                                print("中央センサ停止、左センサ方向空きの為、右に切って下がる")
                                return angle_adj_2 * -1.0 , dis_back_throttle 
                            else:                       #中央が近く、左側が大きく開いている場合、ハンドルを少し右(angle_adj_2 * 1.0)に切って後退
                                print("中央センサ停止、右センサ方向空きの為、左に切って下がる")
                                return angle_adj_2 * 1.0, dis_back_throttle
                else:
                    return pilot_angle, user_throttle * cfg.AI_THROTTLE_MULT #使える？
                return pilot_angle, user_throttle

            else: #local
                return pilot_angle, pilot_throttle * cfg.AI_THROTTLE_MULT
        
    V.add(DriveMode(), 
          inputs=['user/mode', 'user/angle', 'user/throttle',
                  'pilot/angle', 'pilot/throttle', 'distanceLL', 'distanceL','distanceC','distanceR','distanceRR','prev_distanceLL','prev_distanceL','prev_distanceC','prev_distanceR','prev_distanceRR'], 
          outputs=['angle', 'throttle']) 

    
    #to give the car a boost when starting ai mode in a race.
    aiLauncher = AiLaunch(cfg.AI_LAUNCH_DURATION, cfg.AI_LAUNCH_THROTTLE, cfg.AI_LAUNCH_KEEP_ENABLED)
    
    V.add(aiLauncher,
        inputs=['user/mode', 'throttle'],
        outputs=['throttle'])



    if isinstance(ctr, JoystickController):
        ctr.set_button_down_trigger(cfg.AI_LAUNCH_ENABLE_BUTTON, aiLauncher.enable_ai_launch)


    class AiRunCondition:
        '''
        A bool part to let us know when ai is running.
        '''
        def run(self, mode):
            if mode == "user":
                return False
            return True

    V.add(AiRunCondition(), inputs=['user/mode'], outputs=['ai_running'])

    #Ai Recording
    class AiRecordingCondition:
        '''
        return True when ai mode, otherwize respect user mode recording flag
        '''
        def run(self, mode, recording):
            if mode == 'user':
                return recording
            return True

    if cfg.RECORD_DURING_AI:
        V.add(AiRecordingCondition(), inputs=['user/mode', 'recording'], outputs=['recording'])
    
    #Drive train setup
    if cfg.DONKEY_GYM:
        pass

    elif cfg.DRIVE_TRAIN_TYPE == "SERVO_ESC":
        from donkeycar.parts.actuator import PCA9685, PWMSteering, PWMThrottle

        steering_controller = PCA9685(cfg.STEERING_CHANNEL, cfg.PCA9685_I2C_ADDR, busnum=cfg.PCA9685_I2C_BUSNUM)
        steering = PWMSteering(controller=steering_controller,
                                        left_pulse=cfg.STEERING_LEFT_PWM, 
                                        right_pulse=cfg.STEERING_RIGHT_PWM)
        
        throttle_controller = PCA9685(cfg.THROTTLE_CHANNEL, cfg.PCA9685_I2C_ADDR, busnum=cfg.PCA9685_I2C_BUSNUM)
        throttle = PWMThrottle(controller=throttle_controller,
                                        max_pulse=cfg.THROTTLE_FORWARD_PWM,
                                        zero_pulse=cfg.THROTTLE_STOPPED_PWM, 
                                        min_pulse=cfg.THROTTLE_REVERSE_PWM)

        V.add(steering, inputs=['angle'])
        V.add(throttle, inputs=['throttle'])
    

    elif cfg.DRIVE_TRAIN_TYPE == "DC_STEER_THROTTLE":
        from donkeycar.parts.actuator import Mini_HBridge_DC_Motor_PWM
        
        steering = Mini_HBridge_DC_Motor_PWM(cfg.HBRIDGE_PIN_LEFT, cfg.HBRIDGE_PIN_RIGHT)
        throttle = Mini_HBridge_DC_Motor_PWM(cfg.HBRIDGE_PIN_FWD, cfg.HBRIDGE_PIN_BWD)

        V.add(steering, inputs=['angle'])
        V.add(throttle, inputs=['throttle'])
    

    elif cfg.DRIVE_TRAIN_TYPE == "DC_TWO_WHEEL":
        from donkeycar.parts.actuator import TwoWheelSteeringThrottle, Mini_HBridge_DC_Motor_PWM

        left_motor = Mini_HBridge_DC_Motor_PWM(cfg.HBRIDGE_PIN_LEFT_FWD, cfg.HBRIDGE_PIN_LEFT_BWD)
        right_motor = Mini_HBridge_DC_Motor_PWM(cfg.HBRIDGE_PIN_RIGHT_FWD, cfg.HBRIDGE_PIN_RIGHT_BWD)
        two_wheel_control = TwoWheelSteeringThrottle()

        V.add(two_wheel_control, 
                inputs=['throttle', 'angle'],
                outputs=['left_motor_speed', 'right_motor_speed'])

        V.add(left_motor, inputs=['left_motor_speed'])
        V.add(right_motor, inputs=['right_motor_speed'])

    elif cfg.DRIVE_TRAIN_TYPE == "SERVO_HBRIDGE_PWM":
        from donkeycar.parts.actuator import ServoBlaster, PWMSteering
        steering_controller = ServoBlaster(cfg.STEERING_CHANNEL) #really pin
        #PWM pulse values should be in the range of 100 to 200
        assert(cfg.STEERING_LEFT_PWM <= 200)
        assert(cfg.STEERING_RIGHT_PWM <= 200)
        steering = PWMSteering(controller=steering_controller,
                                        left_pulse=cfg.STEERING_LEFT_PWM, 
                                        right_pulse=cfg.STEERING_RIGHT_PWM)
       

        from donkeycar.parts.actuator import Mini_HBridge_DC_Motor_PWM
        motor = Mini_HBridge_DC_Motor_PWM(cfg.HBRIDGE_PIN_FWD, cfg.HBRIDGE_PIN_BWD)

        V.add(steering, inputs=['angle'])
        V.add(motor, inputs=["throttle"])

    
    #add tub to save data

    inputs=['cam/image_array',
            'user/angle', 'user/throttle', 
            'user/mode']

    types=['image_array',
           'float', 'float',
           'str']

    if cfg.TRAIN_BEHAVIORS:
        inputs += ['behavior/state', 'behavior/label', "behavior/one_hot_state_array"]
        types += ['int', 'str', 'vector']
    
    if cfg.HAVE_IMU:
        inputs += ['imu/acl_x', 'imu/acl_y', 'imu/acl_z',
            'imu/gyr_x', 'imu/gyr_y', 'imu/gyr_z']

        types +=['float', 'float', 'float',
           'float', 'float', 'float']

    if cfg.RECORD_DURING_AI:
        inputs += ['pilot/angle', 'pilot/throttle']
        types += ['float', 'float']
    
    th = TubHandler(path=cfg.DATA_PATH)
    tub = th.new_tub_writer(inputs=inputs, types=types, user_meta=meta)
    V.add(tub, inputs=inputs, outputs=["tub/num_records"], run_condition='recording')

    if cfg.PUB_CAMERA_IMAGES:
        from donkeycar.parts.network import TCPServeValue
        from donkeycar.parts.image import ImgArrToJpg
        pub = TCPServeValue("camera")
        V.add(ImgArrToJpg(), inputs=['cam/image_array'], outputs=['jpg/bin'])
        V.add(pub, inputs=['jpg/bin'])

    if type(ctr) is LocalWebController:
        print("You can now go to <your pis hostname.local>:8887 to drive your car.")
    elif isinstance(ctr, JoystickController):
        print("You can now move your joystick to drive your car.")
        #tell the controller about the tub        
        ctr.set_tub(tub)
        
        if cfg.BUTTON_PRESS_NEW_TUB:
    
            def new_tub_dir():
                V.parts.pop()
                tub = th.new_tub_writer(inputs=inputs, types=types, user_meta=meta)
                V.add(tub, inputs=inputs, outputs=["tub/num_records"], run_condition='recording')
                ctr.set_tub(tub)
    
            ctr.set_button_down_trigger('cross', new_tub_dir)
        ctr.print_controls()

    #run the vehicle for 20 seconds
    V.start(rate_hz=cfg.DRIVE_LOOP_HZ, 
            max_loop_count=cfg.MAX_LOOPS)


if __name__ == '__main__':
    args = docopt(__doc__)
    cfg = dk.load_config()
    
    if args['drive']:
        model_type = args['--type']
        camera_type = args['--camera']
        drive(cfg, model_path=args['--model'], use_joystick=args['--js'], model_type=model_type, camera_type=camera_type,
            meta=args['--meta'])
    
    if args['train']:
        from train import multi_train, preprocessFileList
        
        tub = args['--tub']
        model = args['--model']
        transfer = args['--transfer']
        model_type = args['--type']
        continuous = args['--continuous']
        aug = args['--aug']     

        dirs = preprocessFileList( args['--file'] )
        if tub is not None:
            tub_paths = [os.path.expanduser(n) for n in tub.split(',')]
            dirs.extend( tub_paths )

        multi_train(cfg, dirs, model, transfer, model_type, continuous, aug)

