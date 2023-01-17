#! /usr/bin/env python

"""
ROS Node to decide the motor activation depending of the force reading
from the gripper
"""

# Import libraries and messages
import rospy
from std_msgs.msg import Float32, Bool

class MotorActivation():
    def __init__(self):
        self.activation = False
        self.motor_publisher = rospy.Publisher('/motor_activation',Bool,queue_size=5)
        self.force_sub = rospy.Subscriber('/gripper_force',Float32,self.gripperForceCallback,queue_size=5)

    def gripperForceCallback(self,msg):
        """
        Subscriber callback to the gripper force message.
        Compares the force with an activation and deactivation limit to
        publish a motor activation message.
        """

        # Take force data
        force = msg.data

        # Compare the data with the limits
        if (force > 370.0 and self.activation == False):
            self.activation = True
        elif (force < 320.0 and self.activation == True):
            self.activation = False
        
        # Publish the activation message
        motor_msg = Bool()
        motor_msg.data = self.activation
        self.motor_publisher.publish(motor_msg)

if __name__ == '__main__':
    # Initialize the ROS Node
    rospy.init_node('motor_activation')
    motor_activation = MotorActivation()
    rospy.spin()