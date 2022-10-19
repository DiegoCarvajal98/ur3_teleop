#!/usr/bin/env python

import rospy
import actionlib
from std_msgs.msg import Float32
from std_srvs.srv import Empty, EmptyRequest
from robotiq_2f_gripper_msgs.msg import CommandRobotiqGripperFeedback, CommandRobotiqGripperActionResult, CommandRobotiqGripperAction, CommandRobotiqGripperGoal
from robotiq_2f_gripper_control.robotiq_2f_gripper_driver import Robotiq2FingerGripperDriver as Robotiq

class GripperClient():

    def toolAngleCallback(self, msg):
        gripper_openning = (0.085*(msg.data - self.tool_min))/self.tool_range
        rospy.logdebug(msg.data)

        if gripper_openning > self.tool_max:
            gripper_openning = self.tool_max
        elif gripper_openning < self.tool_min:
            gripper_openning = self.tool_min

        Robotiq.goto(self.robotiq_client, pos=gripper_openning, speed=0.1, force=5)

    def calibrationClient(self):
        rospy.wait_for_service('tool_calibration')
        tool_calibration = rospy.ServiceProxy('tool_calibration',Empty)

        tool_request = EmptyRequest()

        return tool_calibration(tool_request)

    def __init__(self):
        self.action_name = rospy.get_param('~action_name', 'command_robotiq_action')
        self.robotiq_client = actionlib.SimpleActionClient(self.action_name, CommandRobotiqGripperAction)
        self.robotiq_client.wait_for_server()

        rospy.loginfo("Starting calibration")
        result = self.calibrationClient()
        rospy.loginfo("Finished calibration")
        self.tool_min = rospy.get_param('/tool_min')
        self.tool_max = rospy.get_param('/tool_max')
        rospy.loginfo(self.tool_min)
        rospy.loginfo(self.tool_max)
        self.tool_range = self.tool_max-self.tool_min

        rospy.Subscriber('tool_angle', Float32, self.toolAngleCallback, queue_size=1)
        rospy.spin()

if __name__ == '__main__':
    rospy.init_node('gripper_client',log_level=rospy.INFO)
    gripper_client = GripperClient()