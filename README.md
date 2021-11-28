# Turtlebot Move
A ROS based obstacle avoiding turtlebot program. Simulation in Gazebo of turtlebot moving randomly and avoiding obstacles.


## Installations

To install ROS refer [this link](http://wiki.ros.org/ROS/Installation).

## Requirements / Assumptions
Project requires and evironment with Ubuntu 18.04, ROS Melodic


### Step 1. Clone repository and build the package
```bash
git clone --recursive https://github.com/kavyadevd/turtlebotMove.git
cp <repository_path> <catkin_workspace_path/src/>
cd <catkin_workspace>
source ./devel/setup.bash
catkin_make
```

### Step 2. Make sure that a roscore is up and running
```bash
roscore
```
