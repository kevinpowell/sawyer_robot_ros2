import numpy as np

import zmq
import time
import json
from robot_library_py import *
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy, JointState
from intera_core_msgs.msg import JointCommand
from std_msgs.msg import Bool
try:
    import thread
except ImportError:
    import _thread as thread
import argparse


class AndroidAsJoy:
    """
    publish same command as physical joystick. (ATTACK 3 in our lab)
    """

    def __init__(self, serverIP, debug=False):  

        self.latest_msg=None 
        self.time_last_ms = time.time_ns() // 1_000_000 
        self.debug=debug

        context = zmq.Context() 
        print("Connecting to hello world server…")
        self.socket = context.socket(zmq.SUB)  
        self.socket.connect(f"tcp://{serverIP}:5555") 
        # Subscribe to all messages (empty string means subscribe to all topics)
        self.socket.setsockopt_string(zmq.SUBSCRIBE, "") 
        print("Subscriber is ready to receive messages...")
 
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



        # # id=data['id']
        # button=data['button']
        # pressed=data['pressed']
        # gx=data['x']
        # gy=data['y']
        # enable_ctrl=data['ctrl']
        # enable_gyro=data['gyro']
        # spinner_angle = self.latest_msg['spinner_angle']
        # spinner_strength = self.latest_msg['spinner_strength']
        # spinner_dx = 50 - self.latest_msg['spinner_x']
        # spinner_dy = 50 - self.latest_msg['spinner_y']
        # xd= np.cos(np.deg2rad(spinner_angle))
        # yd= np.sin(np.deg2rad(spinner_angle))
        
        # # self.grasp = self.latest_msg['gripper']
        # if self.gripper_pressed and not self.latest_msg['gripper']:
        #     self.grasp = not self.grasp
        #     self.gripper_pressed = False
        # elif not self.gripper_pressed and self.latest_msg['gripper']:
        #     self.gripper_pressed = True
        #     self.grasp = not self.grasp
        
        
        # # if self.latest_msg['button']=='gripper' and self.latest_msg['pressed']:
        # #     self.gripper_pressed = True
        # # elif self.latest_msg['button']=='gripper' and not self.latest_msg['pressed']:
        # #     self.gripper_pressed = False
            
            
        # # print(f'button:{button}, pressed:{pressed}, spinner: {xd}, {yd}, grip:{self.grasp}')


        # if not enable_ctrl:
        #     return 

        # if not self.prev_gripper_pressed and self.latest_msg['gripper']: 
        #     self.grasp = not self.grasp
        #     self.prev_gripper_pressed = True
            
        # self.prev_gripper_pressed=pressed
 
        # if enable_gyro:
        #     s=0.005
        #     self.pos[0] += self._pos_step * gx*s  # x
        #     self.pos[1] -= self._pos_step * gy*s  # y

         
        #     self.rot_sensitivity=0.02
        #     # print('xd', xd, spinner_angle, spinner_dx, spinner_dy)

        #     if spinner_dx>80:
        #         self.on_press(AKey('v'))
        #     elif spinner_dx < 20:
        #         self.on_press(AKey('c'))
        #     else: 
        #         self.pos[2] += yd * spinner_strength * 0.00001

        

        # else: 
        #     if spinner_strength > 10:  
        #         self.pos[1] += xd * spinner_strength * 0.00002
        #         self.pos[0] -= yd * spinner_strength * 0.00002


        #     self.pos[2] += gx*0.0005 # z
        

        # if pressed:
        #     if button=='up':
        #         self.on_press(AKey('r'))
        #     elif button=='down':
        #         self.on_press(AKey('f'))
        #     elif button=='Y+':
        #         self.on_press(AKey('a'))
        #     elif button=='Y-':
        #         self.on_press(AKey('d'))
        #     elif button=='X-':
        #         self.on_press(AKey('w'))
        #     elif button=='X+':
        #         self.on_press(AKey('s'))
        #     elif button=='T+':
        #         self.on_press(AKey('c'))
        #     elif button=='T-':
        #         self.on_press(AKey('v'))

    def ws_on_message_old(self, ws, message):  
        self.time_current_ms = time.time_ns() // 1_000_000 
        # print(message) 
        #before pressing button
        # {"id":12435,"type":"common","pressed":false,"x":0.06475485861301422,"y":0.0334872305393219,"enable_ctrl":false,"enable_gyro":false,"f":90}
        #after pressing any button
        # {"id":18343,"type":"common","button":"gripper","pressed":false,"x":0.06506230682134628,"y":0.031077276915311813,"enable_ctrl":false,"enable_gyro":false,"f":90}

        if self.debug:
            print(message)

        data=json.loads(message)
        if 'button' not in data.keys():
            data['button']=''
        self.latest_msg=data

def main(ip, debug):
    # ip='192.168.1.32'
    android = AndroidAsJoy(ip, debug=debug)


    rclpy.init()
    node = Node('AndroidAsJoy') 
    pub_joy = node.create_publisher(Joy, '/joy', 10)
    pub_gripper = node.create_publisher(Bool, '/gripper_command', 2)
    pub_android = node.create_publisher(Bool, '/android_command', 2)
    pub_T = node.create_publisher(Bool, '/android_record', 2)
    freq=50.0
 
    gripper_toggle=False
    

    prev_gripper_pressed=False
    enable_ctrl=False
    last_t=False 
 

    gripper_pressed = False
    grasp=False 

    while True:
        time.sleep(1/freq)

        cmd=Joy()
        cmd.header.stamp = rclpy.clock.Clock().now().to_msg()
        cmd.axes = [0.0]*3
        cmd.buttons = [0]*11

        xyzvel = 0.4 

        data=android.latest_msg
        # print('data=', data)
        if data==None:
            continue 
        

        # id=data['id']
        button=data['button']
        pressed=data['pressed']
        gx=data['x']
        gy=data['y']
        enable_ctrl=data['ctrl']
        enable_gyro=data['gyro']

        is_spinner_mode=False
        if 'spinner_angle' in data:
            is_spinner_mode=True
            spinner_angle = data['spinner_angle']
            spinner_strength = data['spinner_strength']
            spinner_dx = 50 - data['spinner_x']
            spinner_dy = 50 - data['spinner_y']
            xd= np.cos(np.deg2rad(spinner_angle))
            yd= np.sin(np.deg2rad(spinner_angle))
            
             
            if gripper_pressed and not data['gripper']:
                gripper_toggle = not gripper_toggle
                gripper_pressed = False
            elif not gripper_pressed and data['gripper']:
                gripper_pressed = True
                gripper_toggle = not gripper_toggle

            # print('grasp=', grasp, data['gripper']) 

        else:
            if not prev_gripper_pressed and button=='gripper' and pressed:
                gripper_toggle = not gripper_toggle
                # print('toggle')
            prev_gripper_pressed = pressed


        msgT = Bool()
        msgT.data = last_t
        pub_T.publish(msgT) 


        msgB = Bool()
        msgB.data = gripper_toggle
        pub_gripper.publish(msgB)

        msgA = Bool()
        msgA.data = enable_ctrl
        pub_android.publish(msgA) 
        
        if not enable_ctrl: 
            continue

        if enable_gyro: 
            cap=6.0
            gx=min(max(gx, -cap), cap)
            gy=min(max(gy, -cap), cap)
 
            gx=gx/cap*0.3
            gy=gy/cap*0.3

            # print(f'gx={gx:.2f}, gy={gy:.2f}')  

            if abs(gx)<0.05:
                gx=0.0
            if abs(gy)<0.05:
                gy=0.0
 
            cmd.axes[0]=gy 
            cmd.axes[1]=-gx 

            if is_spinner_mode:
                if spinner_dx>80:
                    #z rotate pos
                    pass 
                elif spinner_dx < 20:
                    #z rotate neg
                    pass 
                else: 
                    todo= yd * spinner_strength * 0.004
                    # print('z=', xyzvel, todo) 
                    cmd.axes[2] = todo 
            else:
                # button interface
                pass 
            

        if pressed:
            if button=='up':
                cmd.axes[2] = xyzvel
            elif button=='down':
                cmd.axes[2] = -xyzvel 
            elif button=='Y+':
                cmd.axes[0] = xyzvel
            elif button=='Y-':
                cmd.axes[0] = -xyzvel
            elif button=='X-':
                cmd.axes[1] = xyzvel
            elif button=='X+':
                cmd.axes[1] = -xyzvel


        cmd.buttons[0] = gripper_toggle 
        msgB = Bool()
        msgB.data = gripper_toggle
        pub_gripper.publish(msgB)

        pub_joy.publish(cmd) 
         
        
if __name__=='__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument("-i", "--ip", type=str, help="IP shown on android sixdof app", required=True)
    parser.add_argument('-debug', action='store_true')
    args=parser.parse_args()
    print('args=', args)
    main(args.ip, args.debug)

