"""
Driver class ns Android remote controller.
using socket
"""

import argparse
import datetime
import json
import os
import shutil
import time
from glob import glob

import h5py
import numpy as np

import robosuite as suite
import robosuite.macros as macros
from robosuite import load_controller_config
from robosuite.utils.input_utils import input2action
from robosuite.wrappers import DataCollectionWrapper, VisualizationWrapper

import numpy as np
from pynput.keyboard import Controller, Key, Listener

from robosuite.devices import Device
from robosuite.utils.transform_utils import rotation_matrix

 
import time  
import json 
import zmq

try:
    import thread
except ImportError:
    import _thread as thread

class AKey:
    def __init__(self, char):
        self.char=char



class Android(Device):
    """
    copied from Keyboard class.
    Args:
        pos_sensitivity (float): Magnitude of input position command scaling
        rot_sensitivity (float): Magnitude of scale input rotation commands scaling
    """

    def __init__(self, serverIP, pos_sensitivity=1.0, rot_sensitivity=0.05):

        self._display_controls()
        self._reset_internal_state()

        self._reset_state = 0
        self._enabled = False
        self._pos_step = 0.05

        self.pos_sensitivity = pos_sensitivity
        self.rot_sensitivity = rot_sensitivity

        # # make a thread to listen to keyboard and register our callback functions
        self.listener = Listener(on_press=self.on_press, on_release=self.on_release)

        # start listening
        self.listener.start() 

        context = zmq.Context() 
        print("Connecting to hello world server…")
        self.socket = context.socket(zmq.SUB)  
        self.socket.connect(f"tcp://{serverIP}:5555") 
        # Subscribe to all messages (empty string means subscribe to all topics)
        self.socket.setsockopt_string(zmq.SUBSCRIBE, "") 
        print("Subscriber is ready to receive messages...")

        self.latest_msg=None
        self.time_last_ms = time.time_ns() // 1_000_000

        self.ctrl_freq = 30 #20

        self.prev_gripper_pressed = False
        
        self.last_spinner_dx = 0
        self.last_spinner_dy = 0
        
        self.gripper_pressed = False

        thread.start_new_thread(self.run, ())
        
        
        

    def run(self, *args):
        while True:
            # print('waiting for message')
            message = self.socket.recv_string()
            line = json.loads(message)
            self.ws_on_message(line)


    def ws_on_message(self, data): 
        self.time_current_ms = time.time_ns() // 1_000_000
        dt = self.time_current_ms - self.time_last_ms
        
        # if dt <20:
        #     return
        
         
        if 'button' not in data.keys():
            data['button']=''
        if 'pressed' not in data.keys():
            return
            
        self.latest_msg=data



        # id=data['id']
        button=data['button']
        pressed=data['pressed']
        gx=data['x']
        gy=data['y']
        enable_ctrl=data['ctrl']
        enable_gyro=data['gyro']
        spinner_angle = self.latest_msg['spinner_angle']
        spinner_strength = self.latest_msg['spinner_strength']
        spinner_dx = 50 - self.latest_msg['spinner_x']
        spinner_dy = 50 - self.latest_msg['spinner_y']
        xd= np.cos(np.deg2rad(spinner_angle))
        yd= np.sin(np.deg2rad(spinner_angle))
        
        # self.grasp = self.latest_msg['gripper']
        if self.gripper_pressed and not self.latest_msg['gripper']:
            self.grasp = not self.grasp
            self.gripper_pressed = False
        elif not self.gripper_pressed and self.latest_msg['gripper']:
            self.gripper_pressed = True
            self.grasp = not self.grasp
        
        
        # if self.latest_msg['button']=='gripper' and self.latest_msg['pressed']:
        #     self.gripper_pressed = True
        # elif self.latest_msg['button']=='gripper' and not self.latest_msg['pressed']:
        #     self.gripper_pressed = False
            
            
        # print(f'button:{button}, pressed:{pressed}, spinner: {xd}, {yd}, grip:{self.grasp}')


        if not enable_ctrl:
            return 

        if not self.prev_gripper_pressed and self.latest_msg['gripper']: 
            self.grasp = not self.grasp
            self.prev_gripper_pressed = True
            
        self.prev_gripper_pressed=pressed
 
        if enable_gyro:
            s=0.005
            self.pos[0] += self._pos_step * gx*s  # x
            self.pos[1] -= self._pos_step * gy*s  # y

         
            self.rot_sensitivity=0.02
            # print('xd', xd, spinner_angle, spinner_dx, spinner_dy)

            if spinner_dx>80:
                self.on_press(AKey('v'))
            elif spinner_dx < 20:
                self.on_press(AKey('c'))
            else: 
                self.pos[2] += yd * spinner_strength * 0.00001

        

        else: 
            if spinner_strength > 10:  
                self.pos[1] += xd * spinner_strength * 0.00002
                self.pos[0] -= yd * spinner_strength * 0.00002


            self.pos[2] += gx*0.0005 # z
        

        if pressed:
            if button=='up':
                self.on_press(AKey('r'))
            elif button=='down':
                self.on_press(AKey('f'))
            elif button=='Y+':
                self.on_press(AKey('a'))
            elif button=='Y-':
                self.on_press(AKey('d'))
            elif button=='X-':
                self.on_press(AKey('w'))
            elif button=='X+':
                self.on_press(AKey('s'))
            elif button=='T+':
                self.on_press(AKey('c'))
            elif button=='T-':
                self.on_press(AKey('v'))



    @staticmethod
    def _display_controls():
        """
        Method to pretty print controls.
        """

        def print_command(char, info):
            char += " " * (10 - len(char))
            print("{}\t{}".format(char, info))

        print("")
        print_command("App", "SixDOF") 
        print("")
    
    def reset_grasp(self):
        self.grasp = False
        

    def _reset_internal_state(self):
        """
        Resets internal state of controller, except for the reset signal.
        """
        self.rotation = np.array([[-1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, -1.0]])
        self.raw_drotation = np.zeros(3)  # immediate roll, pitch, yaw delta values from keyboard hits
        self.last_drotation = np.zeros(3)
        self.pos = np.zeros(3)  # (x, y, z)
        self.last_pos = np.zeros(3)
        self.grasp = False

    def start_control(self):
        """
        Method that should be called externally before controller can
        start receiving commands.
        """
        self._reset_internal_state()
        self._reset_state = 0
        self._enabled = True

    def get_controller_state(self):
        """
        Grabs the current state of the keyboard.
        Returns:
            dict: A dictionary containing dpos, orn, unmodified orn, grasp, and reset
        """
        # print('grasp', self.grasp)

        dpos = self.pos - self.last_pos
        self.last_pos = np.array(self.pos)
        raw_drotation = (
            self.raw_drotation - self.last_drotation
        )  # create local variable to return, then reset internal drotation
        self.last_drotation = np.array(self.raw_drotation)
        return dict(
            dpos=dpos,
            rotation=self.rotation,
            raw_drotation=raw_drotation,
            grasp=int(self.grasp),
            reset=self._reset_state,
        )

    def on_press(self, key):
        """
        Key handler for key presses.
        Args:
            key (str): key that was pressed
        """
        # print('onpress', key)

        z_scale=0.05

        try:
            # controls for moving position
            if key.char == "w":
                self.pos[0] -= self._pos_step * self.pos_sensitivity  # dec x
            elif key.char == "s":
                self.pos[0] += self._pos_step * self.pos_sensitivity  # inc x
            elif key.char == "a":
                self.pos[1] -= self._pos_step * self.pos_sensitivity  # dec y
            elif key.char == "d":
                self.pos[1] += self._pos_step * self.pos_sensitivity  # inc y
            elif key.char == "f":
                self.pos[2] -= self._pos_step * self.pos_sensitivity*z_scale  # dec z
            elif key.char == "r":
                self.pos[2] += self._pos_step * self.pos_sensitivity*z_scale  # inc z

            # controls for moving orientation
            elif key.char == "z":
                drot = rotation_matrix(angle=0.1 * self.rot_sensitivity, direction=[1.0, 0.0, 0.0])[:3, :3]
                self.rotation = self.rotation.dot(drot)  # rotates x
                self.raw_drotation[1] -= 0.1 * self.rot_sensitivity
            elif key.char == "x":
                drot = rotation_matrix(angle=-0.1 * self.rot_sensitivity, direction=[1.0, 0.0, 0.0])[:3, :3]
                self.rotation = self.rotation.dot(drot)  # rotates x
                self.raw_drotation[1] += 0.1 * self.rot_sensitivity
            elif key.char == "t":
                drot = rotation_matrix(angle=0.1 * self.rot_sensitivity, direction=[0.0, 1.0, 0.0])[:3, :3]
                self.rotation = self.rotation.dot(drot)  # rotates y
                self.raw_drotation[0] += 0.1 * self.rot_sensitivity
            elif key.char == "g":
                drot = rotation_matrix(angle=-0.1 * self.rot_sensitivity, direction=[0.0, 1.0, 0.0])[:3, :3]
                self.rotation = self.rotation.dot(drot)  # rotates y
                self.raw_drotation[0] -= 0.1 * self.rot_sensitivity
            elif key.char == "c":
                drot = rotation_matrix(angle=0.1 * self.rot_sensitivity, direction=[0.0, 0.0, 1.0])[:3, :3]
                self.rotation = self.rotation.dot(drot)  # rotates z
                self.raw_drotation[2] += 0.1 * self.rot_sensitivity
            elif key.char == "v":
                drot = rotation_matrix(angle=-0.1 * self.rot_sensitivity, direction=[0.0, 0.0, 1.0])[:3, :3]
                self.rotation = self.rotation.dot(drot)  # rotates z
                self.raw_drotation[2] -= 0.1 * self.rot_sensitivity

        except AttributeError as e:
            pass

    def on_release(self, key):
        """
        Key handler for key releases.
        Args:
            key (str): key that was pressed
        """

        try:
            # controls for grasping
            if key == Key.space:
                self.grasp = not self.grasp  # toggle gripper

            # user-commanded reset
            elif key.char == "q":
                self._reset_state = 1
                self._enabled = False
                self._reset_internal_state()

        except AttributeError as e:
            pass
