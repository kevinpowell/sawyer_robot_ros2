import numpy as np
from PIL import Image
 
from matplotlib import pyplot as plt
 
import scipy.io as sio
 
import numpy as np
import cv2 
import pyrealsense2 as rs
import threading 
import time



class MyRealSense:
    def __init__(self):
        self.pipe = rs.pipeline()
        self.profile = self.pipe.start()

    def get_current_frame(self, scale=0.5):
        frames = self.pipe.wait_for_frames()
        color_frame = frames.get_color_frame() 

        image=np.asanyarray(color_frame.get_data())
        image= cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
        
        image = cv2.resize(image, (int(image.shape[1]*scale), int(image.shape[0]*scale) ) , interpolation = cv2.INTER_AREA)
        return image 

    def close(self):
        self.pipe.stop()

cam=MyRealSense()

is_running=True
while is_running:
    st=time.time()
    image0 =cam.get_current_frame()
    
    dt=time.time()-st
    print("fps: ", 1/(time.time()-st), 'dt=',dt)
 

    cv2.imshow("press q to close", image0)
    key = cv2.waitKey(1)
    # Press esc or 'q' to close the image window
    if key & 0xFF == ord('q') or key == 27:
        is_running=False
cv2.destroyAllWindows()
cam.close()
