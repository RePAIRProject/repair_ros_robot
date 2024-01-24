# RePAIR ROS Robot
<img src="images/logo_repair.png" width="150" height="75">


# 1) Description
This repository contains the software to control the simulated and real RePAIR robot.

### Dependencies
- [catkin](https://catkin-tools.readthedocs.io/en/latest/verbs/catkin_build.html)
- [xacro](http://wiki.ros.org/xacro)
- [Xbot2](https://advrhumanoids.github.io/xbot2/master/index.html)
- [Softhand-plugin](https://github.com/vamsikalagaturu/SoftHand-Plugin/tree/synergy_joint)
- [roboticsgroup_upatras_gazebo_plugins](https://github.com/roboticsgroup/roboticsgroup_upatras_gazebo_plugins)
- [gazebo2rviz](https://github.com/andreasBihlmaier/gazebo2rviz)
- [pysdf](https://github.com/andreasBihlmaier/pysdf)
- [realsense] (https://github.com/issaiass/realsense2_description)
- moveit
- install ddynamical reconfigure
 ``` sudo apt-get install ros-noetic-ddynamic-reconfigure```
- realsense2_camera is available as a debian package of ROS distribution. It can be installed by typing:
```sudo apt-get install ros-$ROS_DISTRO-realsense2-camera```
- [realsense_gazebo_plugin](https://github.com/issaiass/realsense_gazebo_plugin/tree/master)

# 2) Installation

- Clone the repository along with the submodules
	```bash
	mkdir -p ~/repair_robot_ws/src && cd ~/repair_robot_ws/src

	git clone --recurse-submodules -j8 https://github.com/RePAIRProject/repair_ros_robot.git
	```
- Install required Python packages
	```bash
	cd ~/repair_robot_ws/src/repair_ros_robot
	pip3 install -r requirements.txt
	```
- Build the workspace
	- source ROS (`source /opt/ros/noetic/setup.bash`) in all terminals
	```bash
	cd ~/repair_robot_ws

	catkin build
	```
	
	<summary>Troubleshooting
	<details>
	If you get errors during build similar to (where package name is some name):

	```bash
	CMake Error at /opt/ros/noetic/share/catkin/cmake/catkinConfig.cmake:83 (find_package):
	Could not find a package configuration file provided by
	"package_name" with any of the following names:

		package_nameConfig.cmake
		package_name-config.cmake

	Add the installation prefix of "package_name" to CMAKE_PREFIX_PATH
	or set "package_name" to a directory containing one of the
	above files.  If "package_name" provides a separate development
	package or SDK, be sure it has been installed.
	```

	Check the list below:
	<h3>Failure for realsense2 (missing "ddynamic_reconfigure")</h3>
	
	From [this issue](https://github.com/IntelRealSense/realsense-ros/issues/812) it looks like it should be installed by running:
	```bash
	sudo apt-get install ros-noetic-ddynamic-reconfigure 
	```

	<h3>Failure for repair_moveit_xbot (missing "moveit_ros_planning")</h3>
	
	Install moveit by
	```
	sudo apt-get install ros-noetic-moveit
	```
	<h3>Failure for repair_moveit_xbot (missing "rviz_visual_tools")</h3>
	
	Install it by
	```
	sudo apt-get install ros-noetic-rviz-visual-tools
	```
	<h3>Failure for repair_moveit_xbot (missing "moveit_visual_tools")</h3>
	
	Install it by
	```
	sudo apt-get install ros-noetic-moveit-visual-tools 
	```
	</details>
	</summary>
	

	- After successful build, source the workspace in all the terminals
	```
	cd ~/repair_robot_ws

	source devel/setup.bash
	```
- For Gazebo:
	- Copy the fragment model folder from ```repair_urdf/sdf/frag3``` to ```~/.gazebo/models/frag3```.
	- Change the path in file `pysdf/src/pysdf/parser.py` line `26` to your `catkin_ws` src path.

- Install XBot2 to use the real robot drivers ([Source](https://advrhumanoids.github.io/xbot2/master/index.html))
	```bash
	sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'

   sudo apt install curl 
   curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -

   sudo apt update && sudo apt install -y \
   ros-noetic-ros-base \
   libgazebo11-dev

   echo ". /opt/ros/noetic/setup.bash" >> ~/.bashrc

   source $HOME/.bashrc
   sudo apt install -y \
   ros-$ROS_DISTRO-urdf ros-$ROS_DISTRO-kdl-parser \
   ros-$ROS_DISTRO-eigen-conversions ros-$ROS_DISTRO-robot-state-publisher ros-$ROS_DISTRO-moveit-core \
   ros-$ROS_DISTRO-rviz ros-$ROS_DISTRO-interactive-markers ros-$ROS_DISTRO-tf-conversions ros-$ROS_DISTRO-tf2-eigen \
   qttools5-dev libqt5charts5-dev qtdeclarative5-dev

   sudo sh -c 'echo "deb http://xbot.cloud/xbot2/ubuntu/$(lsb_release -sc) /" > /etc/apt/sources.list.d/xbot-latest.list'
   wget -q -O - http://xbot.cloud/xbot2/ubuntu/KEY.gpg | sudo apt-key add -  
   sudo apt update
   sudo apt install xbot2_desktop_full

   echo ". /opt/xbot/setup.sh" >> ~/.bashrc
	```

- Setup XBot2 to use the real robot drivers ``` set_xbot2_config ~/repair_robot_ws/src/repair_ros_robot/repair_cntrl/config/repair_basic.yaml```

> For docs on `repair_interface`, go to the [repair_interface](https://github.com/RePAIRProject/repair_ros_robot/tree/main/repair_interface).

# 3) Usage
## RVIZ
### RVIZ visualization with joint sliders

```bash
roslaunch repair_urdf repair_full_slider.launch 
```

### RVIZ visualization without joint sliders

```bash
roslaunch repair_urdf repair_full.launch
```

## Gazebo simulation
### View the robot in Gazebo
```bash
roslaunch repair_gazebo repair_gazebo.launch
```

- You can ignore the following error messages, the model uses position controllers while p gains are only needed for effort controllers
``` [ERROR] [1675347973.116238028]: No p gain specified for pid.  Namespace: /gazebo_ros_control/pid_gains/x_joint``` 

### Motion planning and execution with Moveit and ros_control in Gazebo
```bash
roslaunch repair_gazebo bringup_moveit.launch
```

- You can ignore the following error messages, the model uses position controllers while p gains are only needed for effort controllers ``` [ERROR] [1675347973.116238028]: No p gain specified for pid.  Namespace: /gazebo_ros_control/pid_gains/x_joint``` 

## XBot2
XBot2 is required when you want to control the real robot. Furthermore, there is a dummy mode that can be used to emulate the real robot interface. Using the dummy mode allows to use RVIZ with Moveit with the real robot controls instead of ros_control. Currently, this repository does not support using the dummy mode with Gazebo.

### Dummy mode
- First, you have to configure your .bashrc so that the roscore is running on your local machine. For this purpose, add the following lines to your .bashrc.
	```bash
	export ROS_MASTER_URI=http://{local_IP}:11311
	export ROS_IP={local_IP}
	```

- Then, source your .bashrc and start the roscore in window 1.
	```bash
	roscore
	```

- Start XBot2 in window 2.
	```bash
	xbot2-core --hw dummy
	```

- Now you you have to start the bridge between XBot2 and ROS in window 3.
	```bash
	rosrun repair_moveit_xbot moveit_xbot_bridge_node
	```

- Finally, in window 4 you can start RVIZ and Moveit to control the emulated robot.
	```bash
	roslaunch repair_moveit_xbot bringup_moveit.launch
	```

### Real robot
- First, you have to configure your .bashrc so that the roscore is running on the robot PC. For this purpose, add the following lines to your .bashrc.
	```bash
	export ROS_MASTER_URI=http://{robot_IP}:11311
	export ROS_IP={local_IP}
	```

- Then, source your .bashrc and connect via ssh to the real robot PC. You will need at least 3 remote command windows.
	```bash
	ssh -X {host_name}@{robot_IP}
	```

- Check in remote window 1 that the roscore is running. In case the roscore is not running you can restart it using the system command systemctl ```--user restart roscore.service```
	```bash
	rostopic list
	```

- Then used the same window to start the ecat_master.
	```bash
	ecat_master
	```

- Now start XBot in remote window 2
	```bash
	xbot2-core --hw ec_pos
	```
	This starts the motors to be controllable with position control. Alternatively you can start them in idle mode using ```xbot2-core --hw idle```

- Now you have to manually start the motor cooling fans in remote window 3.
	```bash
	rosservice call /ec_client/set_motors_fan "motor_name: ['']
	position: [0]
	velocity: [0]
	torque: [0]
	amperage: [0]
	homing_position: [0]
	fan: [true]
	led: [false]"
	```

- On your local PC you will need at least 3 command windows. In command window 1, you have to open the robot GUI. Here, you have to move the robot to the home position by clicking on the ```Home``` button.
	```bash
	xbot2-gui
	```

- In local window 2 you have to start the bridge between XBot and Moveit.
	```bash
	rosrun repair_moveit_xbot moveit_xbot_bridge_node
	```

- Afterwards, you can start Moveit in local window 3.
	```bash
	roslaunch repair_moveit_xbot bringup_moveit.launch
	```

### Run the pick and place demo
The goal of the demo is to pick and place a fresco fragment. There are 2 versions of the demo. The manual demo is moving to fixed poses while the moveit demo is using a perception pipeline to determine a grasp pose. Both demos can be run in Gazebo or with the real robot. For this purpose the ```gazebo``` argument has to be set accordingly. The ```side``` argument defines whether the left or the right hand is used to grasp the fragment.

#### Without perception
```bash
roslaunch repair_interface manual_test.launch side:=right gazebo:=false
```

#### With perception
First you have to move the arms and hands out of the field of view of the torso camera. Afterwards you can execute the following script.
```bash
roslaunch repair_interface moveit_test.launch side:=right gazebo:=false
```

In the beginning two windows will pop up which you have to close by pressing the ```q``` button.

### Information about used topics
- To inspect all the topics exposed by xbot2 run ``` rostopic list ```:
- Send commands to the joints (SoftHand excluded) using ```/xbotcore/command``` topic
- Read joint states (SoftHand excluded) using ```/xbotcore/joint_states``` topic
- Send commands to the SoftHans using ```/{left/right}_hand_v1s/synergy_command``` topic, or inspect the state of each finger looking at ```/{left/right}_hand_v1s/{fingername}_state``` topic

# 4) Known Issues
- The URDF does not reflect the real robot yet.
- The planning parameters have to be optimized so that the robot does not stop between subsequent poses.

# 5) Relevant publications
T.B.A.

