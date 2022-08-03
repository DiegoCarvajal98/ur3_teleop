UR3 Robot Teleoperation
===

<p align="center">
    <img src="https://github.com/cambel/ur3/blob/master/wiki/ur3.gif?raw=true" alt="UR3 & Robotiq 85" width="250">
</p>

Custom ROS package for teleoperation of the UR3 Robot with Robotiq 2F-85 gripper

## Simulation in Gazebo

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

Launch the UR3 Robot driver launch file with the robot ip and calibration file:
  ```
  $ roslaunch ur_robot_driver ur3_bringup.launch robot_ip:=<robot_ip> kinematics_config:=$(rospack find ur3_teleoperation)/calibration/my_robot_calibration.yaml
  ```

Launch Rviz for Moveit control and visualization:
  ```
  $ roslaunch ur3_teleoperation ur3_moveit.launch real_robot:=true
  ```

## Realsense D435

Launch the RealSense camera:
  ```
  $ roslaunch realsense2_camera rs_camera.launch filters:=pointcloud initial_reset:=true
  ```

### Record bag file
  ```
  $ rosbag record -O session1.bag --duration=2m rosout tf tf_static camera/depth/color/points camera/color/image_raw
  ```

### Play bag file
  ```
  $ rosbag play session.bag
  ```