#! /usr/bin/env python

import rospy
from std_msgs.msg import Float32, Bool

class MotorActivation():
    def __init__(self):
        self.activation = False
        self.motor_publisher = rospy.Publisher('/motor_activation',Bool,queue_size=5)
        self.force_sub = rospy.Subscriber('/gripper_force',Float32,self.gripperForceCallback,queue_size=5)

    def gripperForceCallback(self,msg):
        force = msg.data
        if (force > float(370.0) & self.activation == False):
            self.activation = True
        elif (force < float(320.0) & self.activation == True):
            self.activation = False
        
        motor_msg = Bool()
        motor_msg.data = self.activation
        self.motor_publisher.publish(motor_msg)

if __name__ == '__main__':
    rospy.init_node('motor_activation')
    motor_activation = MotorActivation()
    rospy.spin()