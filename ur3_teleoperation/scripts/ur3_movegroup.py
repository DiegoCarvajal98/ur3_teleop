#!/usr/bin/env python

import rospy
import sys
import moveit_commander
import moveit_msgs.msg
from geometry_msgs.msg import PoseStamped, Pose
from moveit_commander.conversions import pose_to_list
from math import pi

class UR3MoveGroup(object):
    def arucoPoseCallback(self, msg):
        pose_goal = Pose()
        pose_goal = msg.pose

        self.move_group.set_pose_target(pose_goal)

        plan = self.move_group.go(wait=True)
        self.move_group.stop()
        self.move_group.clear_pose_target()
        
    def __init__(self):
        
        moveit_commander.roscpp_initialize(sys.argv)

        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        group_name = "arm"
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

        self.aruco_pose_subscriber = rospy.Subscriber('/aruco_single/pose', PoseStamped, 
                                                        self.arucoPoseCallback)

if __name__ == '__main__':
    rospy.init_node('ur3_movegroup', anonymous=True)
    ur3_movegroup = UR3MoveGroup()
    rospy.spin()