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


# If you want color back in Terminal
# sudo nano /root/.bashrc
# export PS1="\[\e[1;32m\]\u@\h:\[\e[1;34m\]\w\[\e[0m\]\$ "