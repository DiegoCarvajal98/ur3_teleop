UR3 Robot Teleoperation
===

<p align="center">
    <img src="https://github.com/cambel/ur3/blob/master/wiki/ur3.gif?raw=true" alt="UR3 & Robotiq 85" width="250">
</p>

Custom ROS package for teleoperation of the UR3 Robot with Robotiq 2F-85 gripper

## Gazebo Simulation

Initialize Gazebo simulation:
  ```
  $ roslaunch ur3_teleoperation ur3_gripper_85.launch
  ```

Launch Rviz for Moveit control and visualization:
  ```
  $ roslaunch ur3_teleoperation ur3_moveit.launch
  ```

Run the collision objects node to add the table and UR3 base collision objects to Rviz:
  ```
  $ rosrun ur3_teleoperation collision_objects
  ```

## Real robot control

Look up the computer IP, set the IP on the UR3 configuration

Launch the UR3 Robot driver launch file with the robot ip and calibration file:
  ```
  $ roslaunch ur_robot_driver ur3_bringup.launch robot_ip:=<robot_ip> kinematics_config:=$(rospack find ur3_teleoperation)/calibration/robot_calibration.yaml
  ```

Launch the Robotiq 2F-85 gripper server and the gripper's communication port:
  ```
  $ roslaunch robotiq_2f_gripper_control robotiq_action_server.launch comport:=/dev/ttyUSB0
  ```

To control the robot with Rviz, launch Rviz tool for Moveit:
  ```
  $ roslaunch ur3_teleoperation ur3_moveit.launch real_robot:=true
  ```

## Operator system nodes

* ROSSerial node for haptic tool
* usb_cam node for tool pose capture
* ArUco node for tool pose capture
* rqt_image_view node for user feedback
* Tool calibration server

## Robot system nodes

* Camera to base_link static transform publisher node
* Gripper control client node
* Gripper control server node
* ur_ros_control node
* UR3 moveit commander node
* ROSSerial node for the force sensor
* usb_cam node for teleoperation space visualization