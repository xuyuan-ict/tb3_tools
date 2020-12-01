# tb3_tools
Some extended packages for monitoring and controlling turtlebot3 robot.

## Introduction
tb3_tools is a set of packages to help you better monitor and control your turtlebot3 robot. It includes two packages: _tb3\_safe\_teleop_ and _tb3\_monitor_. 
* The _tb3\_safe\_teleop_ package provides commands for safe teleoperation with different input devices. It use LaserScan infomation to estimated the distance between the robot and obstacles, and stop the robot's movement within a customized safe distance.
* The _tb3\_monitor_ package provides commands to monitor nodes and robot's states in real time.


## Install
```shell
# clone
cd /path/to/your/catkin_ws/src
git clone https://github.com/xuyuan-ict/tb3_tools.git

# build
cd /path/to/your/catkin_ws
catkin_make -DCMAKE_BUILD_TYPE=Release  # Release build is recommended
source devel/setup.bash
```

## Quick Starts
1. _**tb3_safe_teleop**_

    ```shell
    tb3-safe-teleop key [tb3_model] [safe_distance]
    ```
Parameter Descriptions
* _key_ : in current version, we only implement safe teleoperation with keyboard device. Other input devices (e.g. ps3 joystick and xbox360 joystick) will be implemented in the near future.
* _tb3\_model_ : the turtlebo3 model type [burger, waffle, waffle_pi]
* _safe\_distance_ : the safe distance between the robot and obstacles


2. _**tb3\_monitor**_

    ```shell
    tb3-monitor node [all|node_name]
    tb3-monitor state [all|state_name]
    ```
Parameter Descriptions
* _node all_ : list all active nodes with subscribed and published topics per seconds
* _node node\_name_ : list specific active node with subscribed and published topics per seconds
* _state all_ : list all robot's states with subscribed and published topics per seconds
* _state state\_name_ : list specific robot's state (e.g. odometry, goal, velocity) with subscribed and published topics per seconds

