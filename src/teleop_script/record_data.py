
from robot_library_py import *
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy, JointState
from intera_core_msgs.msg import JointCommand

from threading import Thread
import time
import numpy as np 
from std_msgs.msg import Bool
import rotm2euler

from utils_camera import MyRealSense, CVCamera
import cv2
from matplotlib import pyplot as plt
import pickle
import datetime

# !v4l2-ctl --list-devices

cam_wrist=MyRealSense()
cam_front=CVCamera(0)

class ROSInterface:
    def __init__(self, node, robot):
        
        self.robot = robot
        self.node = node
        self.jointMap = {name: ind for ind, name in enumerate(self.robot.jointNames)} 

        self.latest_joint_states = None
        self.latest_gripper_state=False
        self.android_msg=None

    def joint_states_callback(self, msg):
        self.latest_joint_states = msg
        q = self.robot.getJoints()
        # print('joint_states:', q)
        for ind, name in enumerate(msg.name):
            if name in self.jointMap:
                q[self.jointMap[name]] = msg.position[ind]
        self.robot.setJoints(q)
    
    def gripper_callback(self, msg):
        self.latest_gripper_state=msg.data
    def android_callback(self, msg):
        self.android_msg = msg

    def spin_thread(self):
        self.node.create_subscription(JointState, 'robot/joint_states', self.joint_states_callback, 10) 
        self.node.create_subscription(Bool, '/gripper_command', self.gripper_callback, 10)
        self.node.create_subscription(Bool, '/android_record', self.android_callback, 10)
        rclpy.spin(self.node)


rclpy.init()
robot = URDFModel('/home/ns/sawyer_robot_ros2/src/teleop_script/sawyer.urdf')
jointMap = {name: ind for ind, name in enumerate(robot.jointNames)}

node = Node('record_data')
q = robot.getJoints()
ros_interface = ROSInterface(node, robot)

t1 = Thread(target=ros_interface.spin_thread)
t1.start()


msg=ros_interface.latest_joint_states

msgs=[]
imgs=[]
grips=[]
motion_detected=False
stop_count=0

print('Make sure gripper is closed before starting')
print('record will start when gripper open for the first time.')

started=False  #make sure gripper is closed before starting

# st=time.time()
for i in range(1000000):
    image_wrist=cam_wrist.get_current_frame()  
    image_front=cam_front.get_current_frame() 
    msg=ros_interface.latest_joint_states
    gripper_isopen=ros_interface.latest_gripper_state

    if msg==None:
        continue 
    if len(msg.position)==1:
        continue

    if not ros_interface.android_msg ==None: 
        if not ros_interface.android_msg.data: 
            if started:
                print('recording stopped')
                break
            else: 
                print('recording not started')
                continue
        else:
            print('started')
            started=True
            st=time.time()

    # if not started and gripper_isopen:
    #     print('started')
    #     started=True
    #     st=time.time()

    if started:
        msgs.append(msg)
        imgs.append( (image_wrist, image_front) ) 
        grips.append(gripper_isopen)
    

    image=np.concatenate([image_wrist, image_front], axis=1) 
    cv2.imshow('frame',image)
    if cv2.waitKey(1) == 27: 
        print('UI closed')
        break  # esc to quit
cv2.destroyAllWindows() 


print('done')
dt=time.time()-st
print( len(msgs), dt, len(msgs)/dt )

cam_front.close()
cam_wrist.close()



#save data
tosave={}

tosave['msgs']=msgs
tosave['imgs']=imgs
tosave['grips']=grips

dir='/home/ns/data_sawyer/spoon/'

N=len(msgs)
 
now = datetime.datetime.now()
time_str=now.strftime("%m_%d_%Y_%H_%M")
fn=dir+time_str+'_'+str(dt)+'_'+str(N) +'.pkl'
print(f'saving to {fn}')
with open(fn, 'wb') as f:
    pickle.dump(tosave, f)

print('done')
rclpy.shutdown()


