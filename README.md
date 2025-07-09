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
## 🛠 General Installation

1. First, clone the repository and its submodules:

	```bash
	mkdir -p ~/repair_robot_ws/src && cd ~/repair_robot_ws/src

	git clone --recurse-submodules -j8 https://github.com/RePAIRProject/repair_ros_robot.git
	git clone https://github.com/RePAIRProject/repair_motion_controller
	```
2. Download fresco 3D models from [Nextcloud](https://cloud.vi.cs.uni-bonn.de/index.php/s/MsAF7bEkNmRZ2jB) to `src/repair_ros_robot/repair_urdf/sdf`. As of now some Frescos might need their own urdf which should just be copy paste of names. 
3. Download fresco recognition models and placing sequence from [Nextcloud](https://cloud.vi.cs.uni-bonn.de/index.php/s/5AW56qWi5EbY4g8) to `src/repair_ros_robot/repair_interface/sand_detection_models`.


## 🐳 Docker Setup (VS Code)
We provide Docker-based installation instructions compatible with [Visual Studio Code's Dev Containers](https://code.visualstudio.com/docs/devcontainers/containers). However, these steps mostly also apply to standard Docker usage.

> **Prerequisite:** Complete the general installation above before proceeding.

### 1. Enable Docker Display Access

In a terminal, run: ``` xhost +local:docker```

### 2. Install GPU Support

To enable visual outputs via GPU, install the following (if not already present):

```bash
sudo apt-get install -y nvidia-container-toolkit
sudo nvidia-ctk runtime configure --runtime=docker
sudo systemctl restart docker
```

### 3. Prepare the `.devcontainer` Folder
Our work is based on this [ROS1 Docker Container](https://github.com/Eruvae/ROS-devcontainer/tree/main/ros1). Either create a new `.devcontainer` folder or copy the one from the before mentioned ROS1 Docker setup into `~/repair_robot_ws`. Then, replace or add the following example files (provided in this repo) inside `.devcontainer`:

```
example_dockerfile.txt      → Dockerfile  
example_postcreate.sh       → postCreate.sh  
example_devcontainer.json   → devcontainer.json  
requirements.txt
```

### 4. Launch the Container in VS Code

1. Install the **Dev Containers** extension in VS Code.
2. Press `Ctrl + Shift + P` and select ``` Dev Containers: Open Folder in Container```
3. Choose the `~/repair_robot_ws` folder.
4. VS Code will now build the container. To rebuild it later, repeat the same command and select: ```Dev Containers: Rebuild Container```

### 5. Build the ROS Workspace

Once inside the container, build your workspace:

```bash
cd /home/ws
catkin build
```

> 🛠 The **XBot** installation is handled automatically by the `postCreate.sh` script.

#### 💥 In case your memory gets filled up by VsCode when starting it leading to a PcCrash:
- Try to close the Ports in VsCode and or Rebuild your container without cache.

### ✅ Optional: Restore Terminal Colors in Docker

If your Docker terminal lacks color, fix it by adding the following to `/root/.bashrc`:

```bash
sudo nano /root/.bashrc
# Add the following line
export PS1="\[\e[1;32m\]\u@\h:\[\e[1;34m\]\w\[\e[0m\]\$ "
```


## 🖥️ Local Installation (Without Docker)

If you prefer to run the project natively without Docker, follow these steps **after** completing the [General Installation](#-general-installation).

---

### 1. Install Python Dependencies

```bash
cd ~/repair_robot_ws/src/repair_ros_robot
pip3 install -r requirements.txt
```

### 2. Build the ROS Workspace

Ensure you have sourced your ROS environment in every terminal:

```bash
source /opt/ros/noetic/setup.bash
cd ~/repair_robot_ws
catkin build
```

---

### ⚠️ Troubleshooting Build Errors

If you encounter errors like:

```bash
CMake Error at /opt/ros/noetic/share/catkin/cmake/catkinConfig.cmake:83 (find_package):
Could not find a package configuration file provided by "package_name" ...
```

Check the solutions below:

#### 🔧 Missing: `ddynamic_reconfigure` (used by `realsense2`)

```bash
sudo apt-get install ros-noetic-ddynamic-reconfigure
```

#### 🔧 Missing: `moveit_ros_planning` (used by `repair_moveit_xbot`)

```bash
sudo apt-get install ros-noetic-moveit
```

#### 🔧 Missing: `rviz_visual_tools` (used by `repair_moveit_xbot`)

```bash
sudo apt-get install ros-noetic-rviz-visual-tools
```

#### 🔧 Missing: `moveit_visual_tools` (used by `repair_moveit_xbot`)

```bash
sudo apt-get install ros-noetic-moveit-visual-tools
```

---

### 3. Source the Workspace

After a successful build:

```bash
cd ~/repair_robot_ws
source devel/setup.bash
```

### 4. Install XBot2 (Real Robot Driver Support)

Follow the [official XBot2 installation guide](https://advrhumanoids.github.io/xbot2/master/index.html), or run:

```bash
# ROS setup
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt install curl 
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
sudo apt update && sudo apt install -y ros-noetic-ros-base libgazebo11-dev

echo ". /opt/ros/noetic/setup.bash" >> ~/.bashrc
source ~/.bashrc

# Additional ROS and GUI tools
sudo apt install -y \
ros-$ROS_DISTRO-urdf ros-$ROS_DISTRO-kdl-parser \
ros-$ROS_DISTRO-eigen-conversions ros-$ROS_DISTRO-robot-state-publisher ros-$ROS_DISTRO-moveit-core \
ros-$ROS_DISTRO-rviz ros-$ROS_DISTRO-interactive-markers ros-$ROS_DISTRO-tf-conversions ros-$ROS_DISTRO-tf2-eigen \
qttools5-dev libqt5charts5-dev qtdeclarative5-dev

# XBot2 repository setup
sudo sh -c 'echo "deb http://xbot.cloud/xbot2/ubuntu/$(lsb_release -sc) /" > /etc/apt/sources.list.d/xbot-latest.list'
wget -q -O - http://xbot.cloud/xbot2/ubuntu/KEY.gpg | sudo apt-key add -
sudo apt update
sudo apt install xbot2_desktop_full

echo ". /opt/xbot/setup.sh" >> ~/.bashrc
```

### 6. Configure XBot2

To set the XBot2 configuration:

```bash
set_xbot2_config ~/repair_robot_ws/src/repair_ros_robot/repair_cntrl/config/repair_basic.yaml
```

 More Information
For additional details on the interface, refer to the [repair_interface documentation](https://github.com/RePAIRProject/repair_ros_robot/tree/main/repair_interface).
---




# 3.) Usage Guide

## Simple Gazebo Simulation

### 🔍 Inspect the Robot in Gazebo

```bash
roslaunch repair_gazebo repair_gazebo.launch
```

### ⚠️ Known Warnings
- You can safely ignore the following error messages — they pertain to position controllers and missing `p` gains, which are only relevant for effort controllers:
	```
	[ERROR] No p gain specified for pid. Namespace: /gazebo_ros_control/pid_gains/x_joint
	```
- URDF warnings like:
  ```
  [ WARN] Link 'right_hand_v1_2_research_thumb_proximal_link' is not known to URDF.
  ```
  These do not impact functionality. You may not see hand animations in RViz but simulations work fine.

---

## Gazebo with XBot2
To run the overall fresco manipulation pipeline as devoloped on the real robot, simply follow the following steps.

### Controller Based Commands: 1 Terminal, 4 Splits

Run each command in its own split:

```bash
roscore
xbot2-core --hw dummy
roslaunch repair_motion_controller bringup_motion_controller.launch
xbot2-gui
```
⚠️ after launching `xbot2-gui` start `homing` and `ros_control`! the simulation will not run properly otherwise.  
> 🔧 Set the `motion_controller_launch` to use `"dummy"` instead of `"real"`.

---

### Run Simulation Commands:
To run the simulation we offer two options, where option 1. is the prefered one:
1. Single combined launch:

	```bash
	roslaunch repair_gazebo repair_gazebo_gazebo.launch
	```

2. Manual launch of each component:

	```bash
	roslaunch repair_gazebo repair_gazebo.launch
	roslaunch repair_gazebo control_utils.launch
	/bin/python /home/ws/src/repair_ros_robot/repair_gazebo/src/xbot_to_gazebo.py
	/bin/python /home/ws/src/repair_ros_robot/repair_gazebo/src/republisher_xbot_to_hand.py
	```
⚠️  After Launching Gazebo, make sure to enable the play button ▶️ inside Gazebo to run the simulation

### Run Pipeline Commands:
### Load Frescos
To spawn a Fresco piece inside the Gazebo simulation, run:
```bash
/home/ws/src/repair_ros_robot/repair_interface/scripts/launch_fresco.py
```
Everytime this command is repeated, the fresco will be respawned at the same position

### Run Experiments 
To run the experiment run the following commands in two seperate terminals:

Terminal 1:

```bash
/bin/python /home/ws/src/repair_ros_robot/repair_interface/scripts/sand_recognition_gazebo.py
```

Terminal 2:

```bash
/bin/python /home/ws/src/repair_ros_robot/repair_interface/scripts/moveit_multi_fresco_cleaned_gazebo.py
```

---

### ✋ Run Arm Movement Only (No Grasping)

You can disable link attachment logic by removing the functions `attach_links` and `detach_links` in `moveit_multi_fresco_cleaned_gazebo.py`.  
You can hardcode the result of grasping to `True` to bypass grasp simulation.

Use the following code for arm motion:

```python
publish_tf_np(arm_target_pose_np, child_frame='arm_grasp_pose')
self.move_arm(self.arm, arm_target_pose_np)
```

Reset robot to home pose:

```python
self.go_home_pose()
```
---

# 🤖 Real-World Robot Usage Guide

Controlling the real robot requires **XBot2**. A **dummy mode** is also available to emulate the real robot interface — ideal for testing MoveIt and RViz without `ros_control`.  

## 🦾 Real Robot Setup

### 1. Configure `.bashrc`

Set your environment to connect to the robot’s ROS master. Add this to your `.bashrc`:

```bash
export ROS_MASTER_URI=http://{robot_IP}:11311
export ROS_IP={local_IP}
```

Then, source your `.bashrc`:

```bash
source ~/.bashrc
```

---

### 2. Remote Setup on Robot PC (via SSH)

```bash
ssh -X {username}@{robot_IP}
```

#### Terminal 1 – Check/Start roscore
```bash
rostopic list
# If not running:
systemctl --user restart roscore.service
```

#### Terminal 1 – Start EtherCAT Master
```bash
ecat_master
```

#### Terminal 2 – Start XBot2 with Position Control
```bash
xbot2-core --hw ec_pos
# Or for idle mode:
# xbot2-core --hw idle
```

#### Terminal 3 – Start GUI
```bash
xbot2-gui
```

---

### 3. Local PC Setup (3 Terminals)

#### Terminal 1 – Start Motion Controller (MoveIt + Klampt)
```bash
roslaunch repair_motion_controller bringup_motion_controller.launch
```

#### Terminal 2 – Only MoveIt
```bash
roslaunch repair_moveit_xbot bringup_moveit.launch
rosrun repair_interface moveit_client.py
```

#### Terminal 3 – Start Chest-mounted Camera
```bash
roslaunch realsense2_camera demo_pointcloud_new.launch serial_no:=f1061874
```

---

### 4. Fresco Recognition

Download Fresco recognition models and run:

```bash
rosrun sand_recognition_with_orientation.py
```

**Available Models:**

```bash
model_name:="best_3pieces_15epochs_larger_batch.pt"  # Group 89 (robust, 3 classes)
model_name:="best_mix.pt"                            # Group 15 and 29
```

> **Note:**  
> - `best_3pieces_15epochs_larger_batch.pt` is robust but supports only 3 IDs.  
> - `best_g89_15epochs_larger_batch.pt` detects more IDs but may be less reliable. Consider reducing the `conf_debug` threshold (line 299) to increase fragment detection.

---

## 🧩 Pick and Place Demo

Run the multi-fragment pick & place pipeline:

```bash
rosrun repair_interface moveit_multi_fresco_cleaned.py
```

**Note:**  
Use `sh_version` options: `v1_2_research`, `v1_wide`, `mixed_hands`.

---

## 📦 Required Files for Recognition

Ask **Luca Palmieri** for the following resources:

- Frescos `RPf_00123` to `RPf_001266` should be placed in:
  - `repair_ros_robot/repair_urdf/sdf`
  - `/home/.gazebo/models`
- Fragment database directory `fragments_db` should be added to:
  - `/home/.gazebo/`

---

## 📡 ROS Topics

- List all topics:
  ```bash
  rostopic list
  ```

- Send joint commands (excluding SoftHand):
  ```
  /xbotcore/command
  ```

- Read joint states (excluding SoftHand):
  ```
  /xbotcore/joint_states
  ```

- SoftHand commands:
  ```
  /left_hand_v1s/synergy_command
  /right_hand_v1s/synergy_command
  ```

- Finger states:
  ```
  /left_hand_v1s/{fingername}_state
  /right_hand_v1s/{fingername}_state
  ```

## LEGACY MoveIt Configuration

### ➕ Increase Path Resolution
In `repair_moveit_config_v2/config/ompl_planning.yaml`:

```yaml
longest_valid_segment_fraction: 0.00005
```

### 🐢 Adjust Arm Speeds
In `repair_moveit_config_v2/config/joint_limits.yaml`:

```yaml
default_velocity_scaling_factor: 0.1
default_acceleration_scaling_factor: 0.1
```

Alternatively, use the **Motion Planning** tab in RViz.



## 🤖 Real world Usage
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

- Now start XBot in remote window 2:
	```bash
	xbot2-core --hw ec_pos
	```
	This starts the motors to be controllable with position control. Alternatively you can start them in idle mode using ```xbot2-core --hw idle```

- Finally use the following to start the gui:
	```bash
	xbot2-gui
	```
- On your local PC you will need at least 3 command windows.

- First, you can start Moveit and Klampt:
	```bash
 	## Moveit and klampt
	roslaunch repair_motion_controller bringup_motion_controller.launch

 	# only Moveit
	roslaunch repair_moveit_xbot bringup_moveit.launch
 	rosrun repair_interface moveit_client.py
	```
 - In order to run the chest mounted camera, use:
	```bash
	roslaunch realsense2_camera demo_pointcloud_new.launch serial_no:=f1061874
	```

 - Finally, In Order to use the Fresco recognition, run:

	```bash
	rosrun sand_recognition_with_orientation.py
	```
 The following models can be currently used:
 
````bash
	model_name:="best_3pieces_15epochs_larger_batch.pt" # for fresco group 89
 	model_name:="best_mix.pt" # for fresco group 15 and 29
````

**Note:** `best_3pieces_15epochs_larger_batch` recognizes only 3 pieces (1, 17 and 20), but it is more robust (it is trained on 3 classes), while `best_g89_15epochs_larger_batch.pt` can recognize all the pieces in group 89, but it is less robust (so better to lower the confidence threshold (`conf_debug`, line 299) and hope for the best: you can grasp more fragments, but sometimes the id is wrong (which means also that the hand choice could be wrong).
 
 Now the system is set-up, and other code can be started!

### Moveit configuration

- To increase/reduce the number of points for a trajectory, update the following parameter for `arm_1` and `arm_2` in [repair_moveit_config_v2/config/ompl_planning.yaml](repair_moveit_config_v2/config/ompl_planning.yaml)
  
	```yaml
	longest_valid_segment_fraction: 0.00005
	```
- To increase/decrease the velocity of arm joints, update the following parameter in [repair_moveit_config_v2/config/joint_limits.yaml](repair_moveit_config_v2/config/joint_limits.yaml)
	
	```yaml
	default_velocity_scaling_factor: 0.1
	default_acceleration_scaling_factor: 0.1
	```
- Alternatively, velocity and acceleration scaling factors can be updated in the Rviz Motion Planning plugin before planning a path.

### Run the pick and place demo
The goal of the demo is to pick and place any number of fresco fragments. 
```bash
rosrun repair_interface moveit_multi_fresco_cleaned.py
```
Note: sh_version options are [v1_2_research, v1_wide, mixed_hands]


To run recognition, a few files need to be added (ask Luca Palmieri for the files):
- RPf_00123 to RPf_001266 should be added to ```repair_ros_robot/repair_urdf/sdf```
- RPf_00123 to RPf_001266 should also be added to  ```/home/.gazebo/models```
- The fragment database directory ```fragments_db``` should be added to ```/home/.gazebo```

### Information about used topics
- To inspect all the topics exposed by xbot2 run ``` rostopic list ```:
- Send commands to the joints (SoftHand excluded) using ```/xbotcore/command``` topic
- Read joint states (SoftHand excluded) using ```/xbotcore/joint_states``` topic
- Send commands to the SoftHans using ```/{left/right}_hand_v1s/synergy_command``` topic, or inspect the state of each finger looking at ```/{left/right}_hand_v1s/{fingername}_state``` topic

# 4) Known Issues
- 

# 5) Relevant publications
T.B.A.

