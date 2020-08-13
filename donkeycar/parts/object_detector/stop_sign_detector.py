import numpy as np
import cv2
import time
import random
import collections
from edgetpu.detection.engine import DetectionEngine
from edgetpu.basic import edgetpu_utils #Nakagawa
from edgetpu.utils import dataset_utils
from PIL import Image
from matplotlib import cm
import os
import urllib.request


class StopSignDetector(object):
    '''
    Requires an EdgeTPU for this part to work
    This part will run a EdgeTPU optimized model to run object detection to detect a stop sign.
    We are just using a pre-trained model (MobileNet V2 SSD) provided by Google.
    '''

    def download_file(self, url, filename):
        if not os.path.isfile(filename):
            urllib.request.urlretrieve(url, filename)

    def __init__(self, min_score, show_bounding_box, debug=False):
        #Nakagawa Test
        edge_tpus = edgetpu_utils.ListEdgeTpuPaths(
        edgetpu_utils.EDGE_TPU_STATE_UNASSIGNED)
        print("stop sign coral" + str(edge_tpus))

        MODEL_FILE_NAME = "ssd_mobilenet_v2_coco_quant_postprocess_edgetpu.tflite"
        LABEL_FILE_NAME = "coco_labels.txt"

        MODEL_URL = "https://github.com/google-coral/edgetpu/raw/master/test_data/ssd_mobilenet_v2_coco_quant_postprocess_edgetpu.tflite"
        LABEL_URL = "https://dl.google.com/coral/canned_models/coco_labels.txt"

        self.download_file(MODEL_URL, MODEL_FILE_NAME)
        self.download_file(LABEL_URL, LABEL_FILE_NAME)

        self.last_5_scores = collections.deque(np.zeros(5), maxlen=5)
        #self.engine = DetectionEngine(MODEL_FILE_NAME) #Nakagawa
        self.engine = DetectionEngine(MODEL_FILE_NAME,'/sys/bus/usb/devices/2-2') #Nakagawa USB2.0

        self.labels = dataset_utils.read_label_file(LABEL_FILE_NAME)

        self.STOP_SIGN_CLASS_ID = 12 #12 stop sign 0 person

        self.min_score = min_score
        self.show_bounding_box = show_bounding_box
        self.debug = debug

    def convertImageArrayToPILImage(self, img_arr):
        img = Image.fromarray(img_arr.astype('uint8'), 'RGB')

        return img

    '''
    Return an object if there is a traffic light in the frame
    '''
    def detect_stop_sign (self, img_arr):
        img = self.convertImageArrayToPILImage(img_arr)

        ans = self.engine.detect_with_image(img,
                                          threshold=self.min_score,
                                          #keep_aspect_ratio=True,
                                          keep_aspect_ratio=False,
                                          relative_coord=False,
                                          top_k=10000) #Nakagawa
        max_score = 0
        traffic_light_obj = None
        if ans:
            for obj in ans:
                #print("Ans object ID %d %5.3f" % (obj.label_id, obj.score))
                if (obj.label_id == self.STOP_SIGN_CLASS_ID):
                    #Nakagawa
                    print("stop sign detected, score = {}".format(obj.score))

                    if self.debug:
                        print("stop sign detected, score = {}".format(obj.score))
                    if (obj.score > max_score):
                        print(obj.bounding_box)
                        traffic_light_obj = obj
                        max_score = obj.score


        # if traffic_light_obj:
        #     self.last_5_scores.append(traffic_light_obj.score)
        #     sum_of_last_5_score = sum(list(self.last_5_scores))
        #     # print("sum of last 5 score = ", sum_of_last_5_score)

        #     if sum_of_last_5_score > self.LAST_5_SCORE_THRESHOLD:
        #         return traffic_light_obj
        #     else:
        #         print("Not reaching last 5 score threshold")
        #         return None
        # else:
        #     self.last_5_scores.append(0)
        #     return None

        return traffic_light_obj

    def draw_bounding_box(self, traffic_light_obj, img_arr):
        xmargin = (traffic_light_obj.bounding_box[1][0] - traffic_light_obj.bounding_box[0][0]) *0.1

        traffic_light_obj.bounding_box[0][0] = traffic_light_obj.bounding_box[0][0] + xmargin
        traffic_light_obj.bounding_box[1][0] = traffic_light_obj.bounding_box[1][0] - xmargin

        ymargin = (traffic_light_obj.bounding_box[1][1] - traffic_light_obj.bounding_box[0][1]) *0.05

        traffic_light_obj.bounding_box[0][1] = traffic_light_obj.bounding_box[0][1] + ymargin
        traffic_light_obj.bounding_box[1][1] = traffic_light_obj.bounding_box[1][1] - ymargin

        cv2.rectangle(img_arr, tuple(traffic_light_obj.bounding_box[0].astype(int)),
                        tuple(traffic_light_obj.bounding_box[1].astype(int)), (0, 255, 0), 2)

    def run(self, img_arr, throttle, debug=False):
        if img_arr is None:
            return throttle, img_arr

        #img_arr2 = img_arr[0:60, 40:120] #Nakagawa真ん中上だけ注目 H上:H下, W左:W右
        #img_arr2 = img_arr
        img_arr2 = img_arr[10:40, 60:100] #Nakagawa真ん中上だけ注目 H上:H下, W左:W右
        #img_arr3 = cv2.resize(img_arr2,(160,120)) #Nakagawa W,H
        img_arr3 = cv2.resize(img_arr2,(640,480)) #Nakagawa W,H

        # Detect traffic light object
        traffic_light_obj = self.detect_stop_sign(img_arr3)

        if traffic_light_obj:
            if self.show_bounding_box:
                self.draw_bounding_box(traffic_light_obj, img_arr3)
            print("Stop light detected")
            return 0, img_arr3
        else:
            return throttle, img_arr