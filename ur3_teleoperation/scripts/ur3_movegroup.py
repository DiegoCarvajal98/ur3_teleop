#!/usr/bin/env python

import rospy
import sys
import moveit_commander
import moveit_msgs.msg
from geometry_msgs.msg import PoseStamped, Pose
from moveit_commander.conversions import pose_to_list

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
        self.aruco_pose_subscriber = rospy.Subscriber('/aruco_single/pose', PoseStamped, 
                                                        self.arucoPoseCallback)

if __name__ == '__main__':
    rospy.init_node('ur3_movegroup', anonymous=True)
    ur3_movegroup = UR3MoveGroup
    rospy.spin()