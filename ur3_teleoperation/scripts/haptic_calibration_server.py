#!/usr/bin/env python

"""
ROS Node to calibrate the minimum and maximum limits for the haptic tool
"""

# Import libraries and ROS messages
import rospy
from std_srvs.srv import Empty, EmptyResponse
from std_msgs.msg import Float32

class ToolCalibration():
    def __init__(self):
        """
        Define tool angle subscriber and calibration server
        """
        self.__tool_angle_subscriber = rospy.Subscriber('tool_angle', Float32, self.toolAngleCallback)
        self.__server = rospy.Service('tool_calibration',Empty,self.handleToolCalibration)

    def toolAngleCallback(self,msg):
        """
        Subscriber callback for the angle message
        """
        self.__tool_angle = msg.data

    def handleToolCalibration(self,req):
        """
        Tool calibration function.
        Take tool angle and set the minimum limit, wait 5 seconds
        and take the tool angle again to set the maximum limit.
        """
        rospy.loginfo("Set the tool to the minimum opening")
        
        # Wait 5 seconds before reading the tool angle
        start_time = rospy.Time.now().secs

        while(rospy.Time.now().secs - start_time < 5):
            continue
        
        # Read the tool angle and set the minimum limit
        tool_min = self.__tool_angle
        rospy.set_param('/tool_min', tool_min)
        rospy.loginfo('Haptic tool minimum opening set to %d', tool_min)

        rospy.loginfo("Set the tool to the maximum opening")

        # Wait another 5 seconds
        start_time = rospy.Time.now().secs

        while(rospy.Time.now().secs - start_time < 5):
            continue
        
        # Read the tool angle and set the maximum limit
        tool_max = self.__tool_angle
        rospy.set_param('/tool_max', tool_max)
        rospy.loginfo('Haptic tool maximum opening set to %d', tool_max)

        # Return the service response
        return EmptyResponse()

if __name__ == '__main__':
    # Initialize ROS Node
    rospy.init_node('tool_calibration_server')
    tool_calibration = ToolCalibration()
    rospy.spin()