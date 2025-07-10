#!/bin/bash
mkdir -p src
sudo rosdep update
sudo rosdep install --from-paths /home/ws/src --ignore-src -y
sudo chown -R $(whoami) /home/ws/
./initialize_workspace.sh
./compile_release.sh
./merge_compile_commands.sh

pip install ultralytics
pip install vedo


sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'

sudo apt install curl 
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -

sudo apt update && sudo apt install -y \
ros-noetic-ros-base \
libgazebo11-dev

echo ". /opt/ros/noetic/setup.bash" >> ~/.bashrc
echo "export ROS_IP=$(hostname -I | awk '{print $1}')" >> ~/.bashrc
echo "export ROS_MASTER_URI=http://$ROS_IP:11311" >> ~/.bashrc

source ~/.bashrc

sudo apt install -y \
ros-$ROS_DISTRO-urdf ros-$ROS_DISTRO-kdl-parser \
ros-$ROS_DISTRO-eigen-conversions ros-$ROS_DISTRO-robot-state-publisher ros-$ROS_DISTRO-moveit-core \
ros-$ROS_DISTRO-rviz ros-$ROS_DISTRO-interactive-markers ros-$ROS_DISTRO-tf-conversions ros-$ROS_DISTRO-tf2-eigen \
qttools5-dev libqt5charts5-dev qtdeclarative5-dev

sudo sh -c 'echo "deb http://xbot.cloud/xbot2/ubuntu/$(lsb_release -sc) /" > /etc/apt/sources.list.d/xbot-latest.list'
wget -q -O - http://xbot.cloud/xbot2/ubuntu/KEY.gpg | sudo apt-key add -  
sudo apt update
sudo apt install xbot2_desktop_full -y

echo ". /opt/xbot/setup.sh" >> ~/.bashrc

source ~/.bashrc

set_xbot2_config /home/ws/src/repair_ros_robot/repair_cntrl/config/repair_basic.yaml

echo "source /home/ws/devel/setup.bash" >> ~/.bashrc
echo ". /opt/xbot/setup.sh" >> ~/.bashrc

source ~/.bashrc

# If you want color back in Terminal
#sudo nano /root/.bashrc
#export PS1="\[\e[1;32m\]\u@\h:\[\e[1;34m\]\w\[\e[0m\]\$ "
