#!/usr/bin/env python

import rospy
from std_srvs.srv import Empty, EmptyResponse
from std_msgs.msg import Float32

class ToolCalibration():
    def __init__(self):
        self.tool_angle_subscriber = rospy.Subscriber('tool_angle', Float32, self.toolAngleCallback)
        self.server = rospy.Service('tool_calibration',Empty,self.handleToolCalibration)

    def toolAngleCallback(self,msg):
        self.tool_angle = msg.data

    def handleToolCalibration(self,req):
        rospy.loginfo("Set the tool to the minimum opening")

        start_time = rospy.Time.now().secs

        while(rospy.Time.now().secs - start_time < 5):
            continue
        
        tool_min = self.tool_angle
        rospy.set_param('/tool_min', tool_min)
        rospy.loginfo('Haptic tool minimum opening set to %d', tool_min)

        rospy.loginfo("Set the tool to the maximum opening")

        start_time = rospy.Time.now().secs

        while(rospy.Time.now().secs - start_time < 5):
            continue
        
        tool_max = self.tool_angle
        rospy.set_param('/tool_max', tool_max)
        rospy.loginfo('Haptic tool maximum opening set to %d', tool_max)

        return EmptyResponse()

if __name__ == '__main__':
    rospy.init_node('tool_calibration_server')
    tool_calibration = ToolCalibration()
    rospy.spin()
