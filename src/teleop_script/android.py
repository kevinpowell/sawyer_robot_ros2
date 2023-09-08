"""
Driver class ns Android remote controller.
"""

import numpy as np
from pynput.keyboard import Controller, Key, Listener

from robosuite.devices import Device
from robosuite.utils.transform_utils import rotation_matrix

import websocket
import time
import json

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

    def __init__(self, serverIP, pos_sensitivity=1.0, rot_sensitivity=1.0):

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

        # websocket.enableTrace(True)
        self.ws = websocket.WebSocketApp(f"ws://{serverIP}:8080",  on_message = self.ws_on_message,  on_error = self.ws_on_error, on_close = self.ws_on_close)
        self.ws.on_open = self.ws_on_open

        self.latest_msg=None
        self.time_last_ms = time.time_ns() // 1_000_000

        self.ctrl_freq = 50 #20

        thread.start_new_thread(self.run, ())

    def run(self, *args):
            self.ws.run_forever()


    def ws_on_message(self, ws, message): 
        self.time_current_ms = time.time_ns() // 1_000_000
        dt = self.time_current_ms - self.time_last_ms
        
        # if dt<1000.0/self.ctrl_freq: 
        #     return
        
        self.time_last_ms = self.time_current_ms

        try:
            self.latest_msg=json.loads(message)
            type=self.latest_msg['type']
            if 'gyro' in type:
                y=self.latest_msg['x']
                x=self.latest_msg['y']
                test=self.latest_msg['test']

                # if x>2.0:
                #     self.on_press(AKey('s'))
                # elif x<-2.0:
                #     self.on_press(AKey('w'))
                
                # if y>2.0:
                #     self.on_press(AKey('a'))
                # elif y<-2.0:
                #     self.on_press(AKey('d'))

                s=0.05

                if test:
                    self.pos[2] += self._pos_step * x*s  # z
                    return
                
                
                self.pos[0] += self._pos_step * x*s  # x
                self.pos[1] -= self._pos_step * y*s  # y

                return 
            

            data=self.latest_msg['data']
            if data=='X+':
                self.on_press(AKey('s'))
            elif data=='X-':
                self.on_press(AKey('w'))
            elif data=='Y+':
                self.on_press(AKey('a'))
            elif data=='Y-':
                self.on_press(AKey('d'))
            elif data=='up':
                self.on_press(AKey('r'))
            elif data=='down':
                self.on_press(AKey('f'))
            elif data=='gripper':
                self.grasp = not self.grasp


        except:
            self.latest_msg=message

        print(message)

    def ws_on_error(self, ws, error):
        print(error)

    def ws_on_close(self, ws):
        print("### closed ###")

    def ws_on_open(self, ws):
        print('onopen')



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
        z_scale=0.03;

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
