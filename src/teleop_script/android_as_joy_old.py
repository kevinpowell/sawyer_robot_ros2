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
        self.msg_id = 1
        self.latest_data={'id':self.msg_id, 'type':'', 'name':'', 'pressed':False, 'x':0.0, 'y':0.0, 'test':False}

        self.time_last_ms = time.time_ns() // 1_000_000

        self.trick_last_button=''

        self.ctrl_freq = 50 #20
        self.gripper_toggle=False 
 
        thread.start_new_thread(self.run, ())

    def run(self, *args):
        print('run')
        self.ws.run_forever()


    def ws_on_message(self, ws, message): 
        self.msg_id += 1
        self.time_current_ms = time.time_ns() // 1_000_000 
        # print(message)
        # "type":"gyro","x":0.5968780517578125,"y":1.0234956741333008,"test":false}

        self.latest_msg=json.loads(message)
        type=self.latest_msg['type']

        if type=='gyro':
            print('gyro=', self.latest_msg)

        tmp={'id':self.msg_id, 'type':type, 'name':'', 'pressed':False, 'x':0.0, 'y':0.0, 'test':False}
        if 'button' in type:
            tmp['name']=self.latest_msg['data']
            tmp['pressed']=self.latest_msg['pressed']
        else: #gyro
            tmp['y']=self.latest_msg['x']
            tmp['x']=self.latest_msg['y']
            tmp['test']=self.latest_msg['test']
        self.latest_data=tmp #update latest data to be processed by main thread

        # print('latest_data', self.latest_data)



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
    freq=5.0

    id_processed=0 #id of latest data processed
    gripper_toggle=False

    last_button_pressed=''
    last_button_arrival_time=0.0
    last_cmd=Joy()
    while True:
        time.sleep(1/freq)

        cmd=Joy()
        cmd.header.stamp = rclpy.clock.Clock().now().to_msg()
        cmd.axes = [0.0]*3
        cmd.buttons = [0]*11

        xyzvel = 0.4

        

        if android.latest_data['id']>id_processed:  #new data arrived
            data=android.latest_data
            id_processed=data['id']
            if 'gripper' in data['name']:
                # print('------------gripper----------')
                gripper_toggle = not gripper_toggle
                # msgB = Bool()
                # msgB.data = gripper_toggle
                # pub_gripper.publish(msgB)
                # continue
            
            elif 'up' in data['name']:
                cmd.axes[2] = xyzvel if data['pressed'] else 0.0
            elif 'down' in data['name']:
                cmd.axes[2] = -xyzvel if data['pressed'] else 0.0
            elif 'Y+' in data['name']:
                cmd.axes[0] = xyzvel if data['pressed'] else 0.0
            elif 'Y-' in data['name']:
                cmd.axes[0] = -xyzvel if data['pressed'] else 0.0
            elif 'X-' in data['name']:
                cmd.axes[1] = xyzvel if data['pressed'] else 0.0
            elif 'X+' in data['name']:
                cmd.axes[1] = -xyzvel if data['pressed'] else 0.0

            if 'gyro' in data['type']:
                gx=android.latest_data['x']
                gy=android.latest_data['y']

                cap=6.0
                gx=min(max(gx, -cap), cap)
                gy=min(max(gy, -cap), cap)

                #map to -0.7~0.7
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

                # cmd.axes[0] = android.latest_data['x']
                # cmd.axes[1] = android.latest_data['y']
            else:
                #its a button.
                last_button_pressed=data['name'] if data['pressed'] else ''
                last_button_arrival_time = time.time()
        else:
            dt= time.time() - last_button_arrival_time
            # print('no new data b=', last_button_pressed, dt)
            #for some button retain pressed information for some time
            #if time passed, consider it as released
            #Trick to deal with continous button pressed but data comes discontinously.
            if last_button_pressed in ['up', 'down', 'Y+', 'Y-', 'X+', 'X-']:
                 
                if dt>1.0:
                    # last_button_pressed=''
                    #enough time passed, consider it as released
                    pass 
                else:
                    #publish the prevous data.
                    cmd=last_cmd
                    cmd.header.stamp = rclpy.clock.Clock().now().to_msg()


        cmd.buttons[0] = gripper_toggle
        msgB = Bool()
        msgB.data = gripper_toggle
        pub_gripper.publish(msgB)

        pub_joy.publish(cmd)
        last_cmd = cmd 
        

if __name__ == '__main__':
    main()

