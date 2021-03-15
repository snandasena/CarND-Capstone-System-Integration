Project: System Integration(Capstone Project)
---

[![Udacity - Self-Driving Car NanoDegree](https://s3.amazonaws.com/udacity-sdc/github/shield-carnd.svg)](http://www.udacity.com/drive)

### Introduction
<img src="imgs/ final-output.gif " width="500" height="450" />
In this project, we'll be writing ROS nodes to implement core functionality of the autonomous vehicle system, including traffic light detection, control, and waypoint following! We will test our code using a simulator that mimics the functionality on Carla.

#### System Architecture Diagram
<img src="imgs/final-project-ros-graph-v2.png" width="1000" height="600" />

#### Code structure

###### (path_to_project_repo)/ros/src/tl_detector/
This package contains the traffic light detection node: `tl_detector.py`. This node takes in data from the `/image_color`, `/current_pose`, and `/base_waypoints` topics and publishes the locations to stop for red traffic lights to the `/traffic_waypoint` topic.

The `/current_pose` topic provides the vehicle's **current position**, and `/base_waypoints` provides a complete list of **waypoints** the car will be following.

<img src="imgs/tl-detector-ros-graph.png" width="700" height="180" />

###### (path_to_project_repo)/ros/src/waypoint_updater/
This package contains the waypoint updater node: `waypoint_updater.py`. The purpose of this node is to update the target velocity property of each waypoint based on traffic light and obstacle detection data. This node will subscribe to the `/base_waypoints`, `/current_pose`, `/obstacle_waypoint`, and `/traffic_waypoint` topics, and publish a list of waypoints ahead of the car with target velocities to the `/final_waypoints` topic.

<img src="imgs/waypoint-updater-ros-graph.png" width="700" height="180" />

###### (path_to_project_repo)/ros/src/twist_controller/

Carla is equipped with a drive-by-wire (dbw) system, meaning the throttle, brake, and steering have electronic control. This package contains the files that are responsible for control of the vehicle: the node `dbw_node.py` and the file `twist_controller.py`, along with a pid and lowpass filter that you can use in our implementation. The dbw_node subscribes to the `/current_velocity` topic along with the `/twist_cmd` topic to receive target linear and angular velocities. 
Additionally, this node will subscribe to `/vehicle/dbw_enabled`, which indicates if the car is under dbw or driver control. This node will publish **throttle**, **brake**, and **steering** commands to the `/vehicle/throttle_cmd`, `/vehicle/brake_cmd`, and `/vehicle/steering_cmd` topics.

<img src="imgs/dbw-node-ros-graph.png" width="700" height="180" />


###### (path_to_project_repo)/ros/src/styx/
A package that contains a server for communicating with the simulator, and a bridge to translate and publish simulator messages to ROS topics.

###### (path_to_project_repo)/ros/src/styx_msgs/
A package which includes definitions of the custom ROS message types used in the project.

###### (path_to_project_repo)/ros/src/waypoint_loader/
A package which loads the static waypoint data and publishes to `/base_waypoints`.

###### (path_to_project_repo)/ros/src/waypoint_follower/
A package containing code from [Autoware](https://github.com/Autoware-AI/autoware.ai) which subscribes to `/final_waypoints` and publishes target vehicle linear and angular velocities in the form of twist commands to the `/twist_cmd` topic.

### References
* https://www.udacity.com/course/self-driving-car-engineer-nanodegree--nd013
* http://ais.informatik.uni-freiburg.de/teaching/ss18/robotics/index_en.php
* https://github.com/snandasena/path-plnaning-n-localization
* https://thinkautonomous.medium.com/

### Acknowledgments
Big thank you to [Udacity](https://www.udacity.com) for providing the template code and simulator for this project.
