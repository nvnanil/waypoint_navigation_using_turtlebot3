# ENPM809Y - Final Project
```
Name: Naveen Anil
```
Course:  ENPM809Y - C++ programming for Robotics

## Software Requirements
```
Ubuntu 20.04
ROS Galactic
Gazebo11
Turtlebot3_Simulation package
RViz

```
### Running the code
1. *Clone the package group13_final to your ros workspace*
2. *Build and source the workspace*
3. *To edit the parameters*
```
cd ~/ros_ws/src/group13_final code .

```
*and edit the waypoints.params.yaml file*

4. *Launch turtlebot3 in the maze world with RViz using*
```
ros2 launch final_project final_project.launch.py 

```
5. *Run the broadcaster for broadcasting the battery poses using*
```
ros2 run group13_final broadcaster_demo --ros-args --remap use_sim_time:=True

```
6. *Launch the navigation node for maze navigation using*
```
ros2 launch group13_final load_param.py

```
