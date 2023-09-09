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

        self.latest_msg=json.loads(message)

    def ws_on_error(self, ws, error):
        print(error)

    def ws_on_close(self, ws):
        print("### closed ###")

    def ws_on_open(self, ws):
        print('onopen')

#TODO: frequency settings in the android app.
#TODO: publish both the gyro and button data in same msg

def main():
    android = AndroidAsJoy('192.168.1.41')


    rclpy.init()
    node = Node('AndroidAsJoy') 
    pub_joy = node.create_publisher(Joy, '/joy', 10)
    pub_gripper = node.create_publisher(Bool, '/gripper_command', 2)
    freq=50.0
 
    gripper_toggle=False

    last_button_pressed=''
    last_button_arrival_time=0.0
    last_cmd=Joy()

    prev_gripper_pressed=False
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

        if not enable_ctrl:
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

            # if gx>2.0:
            #     cmd.axes[1] = -xyzvel
            # elif gx<-2.0:
            #     cmd.axes[1] = xyzvel

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
        last_cmd = cmd 
        

if __name__ == '__main__':
    main()

