import numpy as np

import websocket
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

    def __init__(self, serverIP): 
        # websocket.enableTrace(True)
        self.ws = websocket.WebSocketApp(f"ws://{serverIP}:8080",  on_message = self.ws_on_message,  on_error = self.ws_on_error, on_close = self.ws_on_close)
        self.ws.on_open = self.ws_on_open

        self.latest_msg=None 
        self.time_last_ms = time.time_ns() // 1_000_000 
 
        thread.start_new_thread(self.run, ())

    def run(self, *args):
        print('run')
        self.ws.run_forever()


    def ws_on_message(self, ws, message):  
        self.time_current_ms = time.time_ns() // 1_000_000 
        # print(message) 
        #before pressing button
        # {"id":12435,"type":"common","pressed":false,"x":0.06475485861301422,"y":0.0334872305393219,"enable_ctrl":false,"enable_gyro":false,"f":90}
        #after pressing any button
        # {"id":18343,"type":"common","button":"gripper","pressed":false,"x":0.06506230682134628,"y":0.031077276915311813,"enable_ctrl":false,"enable_gyro":false,"f":90}

        data=json.loads(message)
        if 'button' not in data.keys():
            data['button']=''
        self.latest_msg=data

    def ws_on_error(self, ws, error):
        print(error)

    def ws_on_close(self, ws):
        print("### closed ###")

    def ws_on_open(self, ws):
        print('onopen')

#TODO: frequency settings in the android app.
#TODO: publish both the gyro and button data in same msg

def main(ip):
    # ip='192.168.1.32'
    android = AndroidAsJoy(ip)


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
        

        id=data['id']
        button=data['button']
        pressed=data['pressed']
        gx=data['x']
        gy=data['y']
        enable_ctrl=data['enable_ctrl']
        enable_gyro=data['enable_gyro']

        if button=='T+' and pressed:
            last_t=True
        elif button=='T-' and pressed:
            last_t=False
        
        msgT = Bool()
        msgT.data = last_t
        pub_T.publish(msgT) 

        if not enable_ctrl:
            msgA = Bool()
            msgA.data = enable_ctrl
            pub_android.publish(msgA) 

            continue


        if not prev_gripper_pressed and button=='gripper' and pressed:
            gripper_toggle = not gripper_toggle
            # print('toggle')
        prev_gripper_pressed = pressed

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


        msgA = Bool()
        msgA.data = enable_ctrl
        pub_android.publish(msgA) 

         
        
if __name__=='__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument("-i", "--ip", type=str, help="IP shown on android sixdof app")
    args=parser.parse_args()
    print('args=', args)
    main(args.ip)

