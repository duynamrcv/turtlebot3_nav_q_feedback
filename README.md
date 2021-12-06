# Turtlebot3 Navigation using Feedback control and Q-Learning
The package present the navigation algorithm using the hybrid control (Linear Feedback + Q-Learning)

## Prerequisite
[ROS Noetic](http://wiki.ros.org/noetic/Installation)<br>
Python 3
```
$ pip install numpy
$ pip install matplotlib
$ pip install itertools
```
TurtleBot3 simulation
```
$ sudo apt install ros-noetic-turtlebot3-simulations
```

## Installation
```
$ cd turtlebot3_ws/src
$ git clone https://github.com/duynamrcv/turtlebot3_nav_q_feedback.git
$ cd ..
$ catkin_make
```
Because the package is designed for the Turtlebot3 Burger:
```
$ echo "export TURTLEBOT3_MODEL=burger" >> ~/.bashrc
$ . ~/.basrhc
```

## Running
Make sure all the paths in the file are correct.
```
$ cd turtlebot3_ws
$ . devel/setup.bash
$  roslaunch turtlebot3_nav_q_feedback navigation.launch
```