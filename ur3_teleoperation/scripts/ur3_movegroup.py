#!/usr/bin/env python

import rospy
import sys
import tf2_ros
import moveit_commander
from moveit_msgs.msg import DisplayTrajectory
from geometry_msgs.msg import Pose
from math import pi

class UR3MoveGroup(object):
        
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
        
        moveit_commander.roscpp_initialize(sys.argv)
        self.tf_buffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tf_buffer)

        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        
        group_name = "arm"
        rate = rospy.Rate(10)
        self.move_group = moveit_commander.MoveGroupCommander(group_name)
        self.display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                        DisplayTrajectory,
                                                        queue_size=10)
        
        joint_goal = self.move_group.get_current_joint_values()
        joint_goal[0] = pi/2
        joint_goal[1] = -pi/2
        joint_goal[2] = pi/2
        joint_goal[3] = 0
        joint_goal[4] = pi/2
        joint_goal[5] = 0

        self.move_group.go(joint_goal, wait=True)
        self.move_group.stop()

        self.pose_goal = Pose()

        rospy.Timer(rospy.Duration(0.1),self.tfListenerCallback)

if __name__ == '__main__':
    rospy.init_node('ur3_movegroup', anonymous=True, log_level=rospy.DEBUG)
    ur3_movegroup = UR3MoveGroup()
    rospy.spin()