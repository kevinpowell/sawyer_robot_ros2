import pickle as pkl
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
import glob
import h5py
import os
import argparse
import glob 
import tqdm

urdf_path = os.getcwd() +'/'+'sawyer.urdf'  #'/home/carl/sawyer_robot_ros2/src/teleop_script/sawyer.urdf'
robot = URDFModel(urdf_path)
jointMap = {name: ind for ind, name in enumerate(robot.jointNames)}

joint_names=['right_j0', 'right_j1', 'right_j2', 'right_j3', 'right_j4', 'right_j5', 'right_j6']
joint_inds=[jointMap[name] for name in joint_names]

def get_ee(jointmsg):
    msg=jointmsg    
    q =  robot.getJoints() 
    for ind, name in enumerate(msg.name):
        if name in jointMap:
            q[jointMap[name]] = msg.position[ind]
    robot.setJoints(q)
    
    # T=robot.getBodyTransform('right_l6');
    T=robot.getBodyTransform('camera_link');
    xyz=T[0:3,3]
    e1 = rotm2euler.rotationMatrixToEulerAngles(T[0:3, 0:3])
    ee6=np.concatenate([xyz, e1])
    return ee6


def extract_data(msgs, imgs, grips):
    '''
    msgs: list of joint msgs
    imgs: list of images
    grips: list of gripper states

    msgs are ros msgs 
    similar to matlab code, robot library (rl) is needed to convert msgs to end effector pose
    '''
    
    poss, vels, ees, ts, imgs_wrist, imgs_front=[], [], [], [], [], []

    for i in range(len(msgs)-1):  #remove the last element, as ee is delta_ee
        msg=msgs[i]
        img_wrist, img_front=imgs[i]

        ee=get_ee(msg)
        next_ee=get_ee(msgs[i+1])
        delta_ee=next_ee -ee

        pos =[msg.position[ind] for ind in joint_inds] 
        vel =[msg.velocity[ind] for ind in joint_inds] 
        t=msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9

        poss.append(pos)
        vels.append(vel)
        ees.append(delta_ee)
        ts.append(t)

        # to rgb
        img_wrist=cv2.cvtColor(img_wrist, cv2.COLOR_BGR2RGB)
        img_front=cv2.cvtColor(img_front, cv2.COLOR_BGR2RGB)
        
        img_wrist=cv2.resize(img_wrist, (84,84))
        img_front=cv2.resize(img_front, (84,84))

        imgs_wrist.append(img_wrist)
        imgs_front.append(img_front)

    poss=np.array(poss)
    vels=np.array(vels)
    ees=np.array(ees)
    ts=np.array(ts)
    imgs_wrist=np.array(imgs_wrist)
    imgs_front=np.array(imgs_front)

    grips =np.array(grips[:-1], dtype=np.int8)  

    return poss, vels, ees, ts, imgs_wrist, imgs_front, grips


def load_demo_pkl(demo_name):
    with open(demo_name, 'rb') as f:
        demo = pkl.load(f)
    msgs=demo['msgs']
    imgs=demo['imgs']
    grips=demo['grips']
    return msgs, imgs, grips

def save_to_robomimic_like_hdf5(hdf5_file_name, demo_no, poss, vels, ees, ts, imgs_wrist, imgs_front, actions):

    demo_group=f"/data/demo_{demo_no}"
    print(f'saving demo {demo_group} to {hdf5_file_name}')
    with h5py.File(hdf5_file_name, 'a') as hf:
        group = hf.create_group(demo_group) 
        group.attrs['num_samples'] = poss.shape[0]
        group.create_dataset('obs/robot0_eef_pos', data=ees)                        #TODO: set actual ee pos, not delta ee
        group.create_dataset('obs/robot0_eye_in_hand_image', data=imgs_wrist)
        group.create_dataset('obs/agentview_image', data=imgs_front)
        group.create_dataset('obs/robot0_joint_pos', data=poss)
        group.create_dataset('obs/robot0_joint_vel', data=vels)
        group.create_dataset('actions', data=actions)
        group.create_dataset('times', data=ts)


def main(dir):
    if dir[-1]!='/':
        dir=dir+"/"
    files=glob.glob(dir+'*.pkl') 
    print('Total ', len(files))

    hdf5_file_name=dir+'demos.hdf5'

    for demo_no in tqdm.tqdm(range(len(files))): 
        demo_name=files[demo_no]
        # print(demo_name)

        msgs, imgs, grips = load_demo_pkl(demo_name)
        poss, vels, ees, ts, imgs_wrist, imgs_front, grips = extract_data(msgs, imgs, grips)
        robomimic_action=np.hstack([ees, grips.reshape(-1,1)])

        save_to_robomimic_like_hdf5(hdf5_file_name, demo_no, poss, vels, ees, ts, imgs_wrist, imgs_front, robomimic_action)





if __name__=='__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument("-f", "--src", type=str, default=0,  help="src directory", required=True) 
    args=parser.parse_args()
    print('args=', args)
    main(args.src)

# python3 demos2hdf5.py -f /home/carl/data_sawyer/block
