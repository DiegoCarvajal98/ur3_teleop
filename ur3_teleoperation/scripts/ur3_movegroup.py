#!/usr/bin/env python

import rospy
import sys
import tf
import moveit_commander
import moveit_msgs.msg
from geometry_msgs.msg import PoseStamped, Pose
from moveit_commander.conversions import pose_to_list
from math import pi

class UR3MoveGroup(object):
        
    def __init__(self):
        
        moveit_commander.roscpp_initialize(sys.argv)
        self.listener = tf.TransformListener()

        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        self.pose_goal = Pose()
        group_name = "arm"
        rate = rospy.Rate(10)
        self.move_group = moveit_commander.MoveGroupCommander(group_name)
        self.display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                        moveit_msgs.msg.DisplayTrajectory,
                                                        queue_size=20)
        
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
                (trans,rot) = self.listener.lookupTransform('/marker_frame','/base', rospy.Time(0))
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue
            
            self.pose_goal.position.x = trans[0]
            self.pose_goal.position.y = trans[1]
            self.pose_goal.position.z = trans[2]

            self.pose_goal.orientation.x = rot[0]
            self.pose_goal.orientation.y = rot[1]
            self.pose_goal.orientation.z = rot[2]
            self.pose_goal.orientation.w = rot[3]

            self.move_group.set_pose_target(self.pose_goal)

            plan = self.move_group.go(wait=True)
            self.move_group.stop()
            self.move_group.clear_pose_target()

            rate.sleep()


if __name__ == '__main__':
    rospy.init_node('ur3_movegroup', anonymous=True)
    ur3_movegroup = UR3MoveGroup()
    rospy.spin()