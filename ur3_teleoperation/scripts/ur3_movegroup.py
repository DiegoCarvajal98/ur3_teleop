#!/usr/bin/env python

import rospy
import sys
import tf2_ros
import moveit_commander
from moveit_msgs.msg import DisplayTrajectory
from geometry_msgs.msg import Pose
from ur_dashboard_msgs.srv import Load, LoadRequest
from std_srvs.srv import Trigger, TriggerRequest
from math import pi

class UR3MoveGroup(object):
    def shutdown(self):
        stop_req = TriggerRequest()
        self.stop_prog.call(stop_req)
        rospy.logdebug("SHUTTING DOWN")
        
    def tfListenerCallback(self, event):
        if self.tf_buffer.can_transform('base_link','marker_frame',rospy.Time(0)):
            trans = self.tf_buffer.lookup_transform('base_link','marker_frame',rospy.Time(0))

            rospy.logdebug(trans)

            self.pose_goal.position.x = trans.transform.translation.x
            self.pose_goal.position.y = trans.transform.translation.y
            self.pose_goal.position.z = trans.transform.translation.z
            self.pose_goal.orientation.x = trans.transform.rotation.x
            self.pose_goal.orientation.y = trans.transform.rotation.y
            self.pose_goal.orientation.z = trans.transform.rotation.z
            self.pose_goal.orientation.w = trans.transform.rotation.w

            waypoint = []

            waypoint.append(self.pose_goal)

            rospy.loginfo("Planning path")

            (plan, fraction) = self.move_group.compute_cartesian_path(waypoint,
                                                0.01,
                                                0.0)

            rospy.logdebug("Executing plan")

            self.move_group.execute(plan, wait=True)
            self.move_group.stop()

        else:
            rospy.logdebug("Can't transform")

    def __init__(self):
        rospy.on_shutdown(self.shutdown)
        
        moveit_commander.roscpp_initialize(sys.argv)
        self.tf_buffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tf_buffer)

        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        
        group_name = "arm"
        self.move_group = moveit_commander.MoveGroupCommander(group_name)
        self.display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                        DisplayTrajectory,
                                                        queue_size=10)

        # Start UR3 robot
        rospy.wait_for_service("ur_hardware_interface/dashboard/load_program")
        self.load_prog =  rospy.ServiceProxy("ur_hardware_interface/dashboard/load_program", Load)
        load_req = LoadRequest()
        load_req.filename = "ROS_control.urp"
        self.load_prog.call(load_req)

        rospy.wait_for_service("ur_hardware_interface/dashboard/play")
        self.play_prog =  rospy.ServiceProxy("ur_hardware_interface/dashboard/play", Trigger)
        play_req = TriggerRequest()
        self.play_prog.call(play_req)

        rospy.wait_for_service("ur_hardware_interface/dashboard/stop")
        self.stop_prog =  rospy.ServiceProxy("ur_hardware_interface/dashboard/stop", Trigger)
        
        joint_goal = self.move_group.get_current_joint_values()
        joint_goal[0] = -pi/2
        joint_goal[1] = -pi/2
        joint_goal[2] = -pi/2
        joint_goal[3] = 0
        joint_goal[4] = pi/2
        joint_goal[5] = pi

        self.move_group.go(joint_goal, wait=True)
        self.move_group.stop()

        self.pose_goal = Pose()

        rate = rospy.Rate(3)

        while not rospy.is_shutdown():
            if self.tf_buffer.can_transform('base_link','marker_filtered',rospy.Time(0)):
                trans = self.tf_buffer.lookup_transform('base_link','marker_filtered',rospy.Time(0))

                rospy.logdebug(trans)

                self.pose_goal.position.x = trans.transform.translation.x
                self.pose_goal.position.y = trans.transform.translation.y
                self.pose_goal.position.z = trans.transform.translation.z
                self.pose_goal.orientation.x = trans.transform.rotation.x
                self.pose_goal.orientation.y = trans.transform.rotation.y
                self.pose_goal.orientation.z = trans.transform.rotation.z
                self.pose_goal.orientation.w = trans.transform.rotation.w

                waypoint = []

                waypoint.append(self.pose_goal)

                rospy.loginfo("Planning path")

                (plan, fraction) = self.move_group.compute_cartesian_path(waypoint,
                                                    0.01,
                                                    0.0)

                rospy.logdebug("Executing plan")

                self.move_group.execute(plan, wait=True)
                self.move_group.stop()

                rate.sleep()

            else:
                rospy.logdebug("Can't transform")
                rate.sleep()

if __name__ == '__main__':
    rospy.init_node('ur3_movegroup', anonymous=True, log_level=rospy.INFO)
    ur3_movegroup = UR3MoveGroup()
    rospy.spin()