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

robot = URDFModel('/home/ns/sawyer_robot_ros2/src/teleop_script/sawyer.urdf')
jointMap = {name: ind for ind, name in enumerate(robot.jointNames)}

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

def pub_vel(vel_pub, qd):
    cmd_msg = JointCommand()
    cmd_msg.names = robot.jointNames
    cmd_msg.mode = cmd_msg.VELOCITY_MODE
    cmd_msg.header.stamp = rclpy.clock.Clock().now().to_msg()
    cmd_msg.velocity =qd
    vel_pub.publish(cmd_msg)

def goto_xyzrpy(ros_interface, target, vel_pub, delay=0.1):
    ee6=get_ee(ros_interface.latest_joint_states)
    xd=ee6-target

    alpha = .0001
    I = np.eye(ros_interface.robot.getJoints().shape[0])

    while np.linalg.norm(xd)>0.1:
        ee6=get_ee(ros_interface.latest_joint_states)
        xd=target-ee6 

        J = robot.getJacobian('camera_link')
        
        qd = np.linalg.inv(J.T @ J + alpha * I) @ J.T @ xd

        pub_vel(vel_pub, list(qd)) 
        time.sleep(delay)

def goto_jointpos(ros_interface, target_jointpos, vel_pub,  alpha=0.7, tol=0.1,  delay=0.1):
    cpos=np.array(ros_interface.latest_joint_states.position)
    dpos=np.linalg.norm(target_jointpos-cpos)

    while dpos>tol: 
        cpos=np.array(ros_interface.latest_joint_states.position)
        dpos=np.linalg.norm(target_jointpos-cpos)
 
        qd =(target_jointpos-cpos)*alpha

        pub_vel(vel_pub, list(qd)) 
        time.sleep(delay)

