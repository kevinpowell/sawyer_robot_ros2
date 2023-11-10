import numpy as np
from PIL import Image
 
from matplotlib import pyplot as plt
 
import scipy.io as sio
 
import numpy as np
import cv2 
import pyrealsense2 as rs
from threading import Thread
import time


class MyRealSense:
    def __init__(self):
        self.pipeline = rs.pipeline() 
        config = rs.config() 
        config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30) 
        self.profile =self.pipeline.start(config)
 
        self.isalive=True
        self.current_frame=None 

    def get_current_frame(self, scale=None):
        frames = self.pipeline.wait_for_frames()
        color_frame = frames.get_color_frame() 

        image=np.asanyarray(color_frame.get_data())
        # image= cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
        if scale!=None:
            image = cv2.resize(image, (int(image.shape[1]*scale), int(image.shape[0]*scale) ) , interpolation = cv2.INTER_AREA)
        return image 
    
    def run(self):
        while self.isalive:
            self.current_frame=self.get_current_frame() 

    def close(self):
        self.isalive=False
        self.pipeline.stop()

class CVCamera:
    def __init__(self, camera_id=0):
        self.camera_id=camera_id

        self.cap = cv2.VideoCapture(camera_id)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

        r, frame = self.cap.read()
        # print('Resolution: ' + str(frame.shape[0]) + ' x ' + str(frame.shape[1]))

        self.isalive=True
        self.current_frame=None
        
    def get_current_frame(self, scale=None):
        ret, image = self.cap.read()
        if scale!=None:
            image = cv2.resize(image, (int(image.shape[1]*scale), int(image.shape[0]*scale) ) , interpolation = cv2.INTER_AREA)
        return image 

    def run(self):
        while self.isalive:
            ret, frame = self.cap.read()
            self.current_frame=frame

    def close(self):
        self.isalive=False
        self.cap.release()