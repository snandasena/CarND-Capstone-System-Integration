Project: System Integration(Capstone Project)
---

[![Udacity - Self-Driving Car NanoDegree](https://s3.amazonaws.com/udacity-sdc/github/shield-carnd.svg)](http://www.udacity.com/drive)

### Introduction
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

