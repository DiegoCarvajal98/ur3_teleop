#!/usr/bin/env python

import rospy
import sys
import tf2_ros
import moveit_commander
import moveit_msgs.msg
from geometry_msgs.msg import PoseStamped, Pose
from sensor_msgs.msg import Image
from moveit_commander.conversions import pose_to_list
from math import pi

class UR3MoveGroup(object):
        
    def __init__(self):
        
        moveit_commander.roscpp_initialize(sys.argv)
        self.tf_buffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tf_buffer)

        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        
        group_name = "arm"
        rate = rospy.Rate(10)
        self.move_group = moveit_commander.MoveGroupCommander(group_name)
        self.display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                        moveit_msgs.msg.DisplayTrajectory,
                                                        queue_size=10)
        # self.aruco_subscriber = rospy.Subscriber('/aruco_single/result', Image, arucoCallback,queue_size=5)
        
        joint_goal = self.move_group.get_current_joint_values()
        joint_goal[0] = pi/2
        joint_goal[1] = -pi/2
        joint_goal[2] = pi/2
        joint_goal[3] = 0
        joint_goal[4] = pi/2
        joint_goal[5] = pi

        self.move_group.go(joint_goal, wait=True)
        self.move_group.stop()

        while not rospy.is_shutdown():
            try:
                if self.tf_buffer.can_transform('shoulder_link','base_link_inertia',rospy.Time(0)):
                    trans = self.tf_buffer.lookup_transform('shoulder_link','base_link_inertia',rospy.Time(0))
                else:
                    rospy.logdebug("Can't transform")
                    rate.sleep()
                    continue
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                rate.sleep()
                continue
            
            self.pose_goal = self.move_group.get_current_pose().pose

            rospy.logdebug("px: %d, py: %d, pz: %d", trans.transform.translation.x, 
                trans.transform.translation.y, trans.transform.translation.z)
            rospy.logdebug("rx: %d, ry: %d, rz: %d", trans.transform.rotation.x, 
                trans.transform.rotation.y, trans.transform.rotation.z)
            # self.pose_goal.position.x = trans[0]
            # self.pose_goal.position.y = trans[1]
            # self.pose_goal.position.z = trans[2]

            # self.pose_goal.orientation.x = rot[0]
            # self.pose_goal.orientation.y = rot[1]
            # self.pose_goal.orientation.z = rot[2]
            # self.pose_goal.orientation.w = 1
# 
            # self.move_group.set_pose_target(self.pose_goal)
# 
            # rospy.logdebug("Planning path")
# 
            # plan = self.move_group.go(wait=True)
# 
            # rospy.logdebug("Plan executed")
            # self.move_group.stop()
            # self.move_group.clear_pose_targets()

            rate.sleep()


if __name__ == '__main__':
    rospy.init_node('ur3_movegroup', anonymous=True, log_level=rospy.DEBUG)
    ur3_movegroup = UR3MoveGroup()
    rospy.spin()