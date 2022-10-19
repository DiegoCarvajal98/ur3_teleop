#! /usr/bin/env python

import rospy
from std_msgs.msg import Float32, Bool

class MotorActivation():
    def __init__(self):
        self.activation = False
        self.force_sub = rospy.Subscriber('gripper_force',Float32,self.gripperForceCallback,queue_size=1)
        self.motor_publisher = rospy.Publisher('motor_activation',Bool,queue_size=1)

    def gripperForceCallback(self,msg):
        force = msg.data
        if (force > 400.0 & self.activation == False):
            self.activation = True
        elif (force < 300.0 & self.activation == True):
            self.activation = False
        
        motor_msg = Bool()
        motor_msg.data = self.activation
        self.motor_publisher.publish(motor_msg)

if __name__ == '__main__':
    rospy.init_node('motor_activation')
    motor_activation = MotorActivation()
    rospy.spin()