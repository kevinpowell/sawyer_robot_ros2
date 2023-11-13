### installation: 
* install rl from source
* inside sawyer_robot_ros2/src
* git clone git@github.com:AssistiveRoboticsUNH/sawyer_robot_ros2.git
* colcon build
* source install/setup.bash

### run:
```
cd /home/ns/sawyer_robot_ros2/sawyer-noetic
docker compose up
```
* run teleop publisher.


```
cd ~/sawyer_robot_ros2/
source install/setup.bash
ros2 run joy joy_node
```


```
cd ~/sawyer_robot_ros2/
source install/setup.bash
ros2 topic echo /joy  (should show all 0s)
```

* run teleop script
```
cd ~/sawyer_robot_ros2/
source install/setup.bash
cd src/teleop_script
(if error: LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/local/lib/)
python3 teleop.py

 

ros2 topic echo /robot/joint_states  (if couldn't determine topic, then run pub_dbg.py then run this command again)


```

### run simulator 
```
cd /home/ns/sawyer_robot_ros2/sawyer_sim
./sawyer2.x86_64
```

### enable gripper listener
```
sudo docker run --privileged -it --net=host pac48/sawyer_demo:latest bash

export ROS_MASTER_URI=http://192.168.1.10:11311
export ROS_IP=192.168.1.30
source devel/setup.bash

rosrun intera_interface enable_robot.py -e

rosrun intera_examples gripper_keyboard.py
<esc>
 
rosrun gripper gripper_listener.py

```


### Run cameras

Realsense install: https://github.com/IntelRealSense/realsense-ros
usb_camera install: https://github.com/ros-drivers/usb_cam/tree/ros2

```
ros2 run usb_cam usb_cam_node_exe --ros-args --remap __ns:=/front_camera --params-file params_1.yaml

ros2 launch realsense2_camera rs_launch.py camera_namespace:=robot1 camera_name:=D455_1 rgb_camera.profile:=640x480x30
```


