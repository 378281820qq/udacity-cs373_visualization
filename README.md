# cs373_visualization

Visualization of algorithms I've learned in Udacity CS373: Artificial Intelligent for Robotics

Programs are modified from my [CS373 coursework](https://github.com/ShinYuFish/udacity-cs373_AI-for-Robotics)

This project:

- based on [ROS](http://wiki.ros.org/) as framework, `Python` as programming language 
- result drawing is shown in [Rviz](http://wiki.ros.org/rviz)
- text results shown in log file (as course programming assignment)

## How to Run
### Environment Settings
Ubuntu16.04 + ROS Kinetic Kame

### Compile in a Local Machine
1. Download this repository, put it into `\src` folder of your ROS workspace
2. `catkin_make` your workspace
3. Source your workspace before running the code (If you've changed setting in `~\.bashrc` file, remember to open a new terminal windows)

### Path Planning
- print path(dijkstra algorithm):
```
roslaunch cs373_visualization print_path_visualize.launch
```
- a_star algorithm:
```
roslaunch cs373_visualization a_star_visualize.launch
```

### Path Smoothing
- path_smoothing:
```
roslaunch cs373_visualization path_smoothing_visualize.launch
```
```
roslaunch cs373_visualization smoothing.launch smooth_type:=path_smoothing

```
- cyclic_smoothing (circle path):
```
roslaunch cs373_visualization cyclic_smoothing_visualize.launch 
```
```
roslaunch cs373_visualization smoothing.launch smooth_type:=cyclic_smoothing

```
- constrained smoothing (fixed points with given path):
```
roslaunch cs373_visualization constrained_smoothing_visualize.launch 
```
```
roslaunch cs373_visualization smoothing.launch smooth_type:=constrained_smoothing

```
### Runaway Robot
There're 2 programs:
- estimate one runaway robot
```
roslaunch cs373_visualiztion runaway_part1.launch
```
- chasing runaway robot with a hunter robot + measurement noise during position estimation
```
roslaunch cs373_visualiztion runaway_part3.launch
```