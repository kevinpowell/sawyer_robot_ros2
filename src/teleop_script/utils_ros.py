
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

import cv2
from matplotlib import pyplot as plt
import pickle
import datetime
import os 
import argparse
from sensor_msgs.msg import Image

'''
To be used in both data collection and inference
'''

class ROSInterface:
    def __init__(self, node, robot):
        
        self.robot = robot
        self.node = node
        self.jointMap = {name: ind for ind, name in enumerate(self.robot.jointNames)} 

        self.latest_joint_states = None
        self.latest_gripper_state=False
        self.record_msg=None

        self.mat = None
        self.mat2= None 
        self.image_wrist=np.zeros([480,640, 3], dtype=np.uint8)
        self.image_front=np.zeros([480,640, 3], dtype=np.uint8)


    def image_callback(self, msg):
        sz = (msg.height, msg.width)
        # print(msg.header.stamp)

        dirty = (self.mat is None or msg.width != self.mat.shape[1] or
                    msg.height != self.mat.shape[0] or len(self.mat.shape) < 2 or
                    self.mat.shape[2] != 3)
        if dirty:
            self.mat = np.zeros([msg.height, msg.width, 3], dtype=np.uint8)
        self.mat[:, :, 2] = np.array(msg.data[0::3]).reshape(sz)
        self.mat[:, :, 1] = np.array(msg.data[1::3]).reshape(sz)
        self.mat[:, :, 0] = np.array(msg.data[2::3]).reshape(sz)
        self.image_wrist=self.mat 

    def image_callback2(self, msg):
        sz = (msg.height, msg.width)
        # print(msg.header.stamp)

        dirty = (self.mat2 is None or msg.width != self.mat2.shape[1] or
                    msg.height != self.mat2.shape[0] or len(self.mat2.shape) < 2 or
                    self.mat2.shape[2] != 3)
        if dirty:
            self.mat2 = np.zeros([msg.height, msg.width, 3], dtype=np.uint8)
        self.mat2[:, :, 2] = np.array(msg.data[0::3]).reshape(sz)
        self.mat2[:, :, 1] = np.array(msg.data[1::3]).reshape(sz)
        self.mat2[:, :, 0] = np.array(msg.data[2::3]).reshape(sz)
        self.image_front=self.mat2 

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
        self.record_msg = msg

    def spin_thread(self):
        self.node.create_subscription(JointState, 'robot/joint_states', self.joint_states_callback, 10) 
        self.node.create_subscription(Bool, '/gripper_command', self.gripper_callback, 10)
        self.node.create_subscription(Bool, '/android_record', self.android_callback, 10)

        self.node.create_subscription(Image,'/D455_1/color/image_raw',self.image_callback,100)
        self.node.create_subscription(Image,'/front_camera/image_raw',self.image_callback2,100)

        rclpy.spin(self.node)

