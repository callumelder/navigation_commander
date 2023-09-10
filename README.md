# navigation_commander

## Cloning
clone into your turtlebot3 workspace's src folder
e.g.
```
cd ~/turtlebot3_ws/src
```
clone the repo
```
git clone https://github.com/callumelder/navigation_commander.git
```

## Modify ~/.bashrc
This is so you don't have to source everytime you open a new terminal

open ~/.bashrc in vscode
```
code ~/.bashrc
```
copy and paste to file (check paths are correct)
```
# ros 2 initialise
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=30 #TURTLEBOT3
source ~/turtlebot3_ws/install/setup.bash
export GAZEBO_MODEL_PATH=~/.gazebo/models:$GAZEBO_MODEL_PATH
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:~/turtlebot3_ws/src/turtlebot3_simulations/turtlebot3_gazebo/models
export TURTLEBOT3_MODEL=waffle_pi
```
