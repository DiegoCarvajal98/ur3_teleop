#! /usr/bin/env python
import rospy
import tf2_ros
from geometry_msgs.msg import TransformStamped, PoseStamped
from simple_kf import SimpleKalmanFilter

class PoseKF():
    def __init__(self):
        self.filt_ori_x = SimpleKalmanFilter(0.5,0.5,0.5)
        self.filt_ori_y = SimpleKalmanFilter(0.5,0.5,0.5)
        self.filt_ori_z = SimpleKalmanFilter(0.5,0.5,0.5)
        self.filt_ori_w = SimpleKalmanFilter(0.5,0.5,0.5)

        self.aruco_msg = TransformStamped()
        self.tf_br = tf2_ros.TransformBroadcaster()
        self.aruco_subscriber = rospy.Subscriber('/aruco_single/pose',PoseStamped,self.arucoCallback,queue_size=5)

    def arucoCallback(self,msg):
        self.aruco_msg.transform.translation.x = msg.pose.position.x
        self.aruco_msg.transform.translation.y = msg.pose.position.y
        self.aruco_msg.transform.translation.z = msg.pose.position.z
        self.aruco_msg.transform.rotation.x = self.filt_ori_x.predict(msg.pose.orientation.x)
        self.aruco_msg.transform.rotation.y = self.filt_ori_y.predict(msg.pose.orientation.y)
        self.aruco_msg.transform.rotation.z = self.filt_ori_z.predict(msg.pose.orientation.z)
        self.aruco_msg.transform.rotation.w = self.filt_ori_w.predict(msg.pose.orientation.w)
        self.aruco_msg.header.frame_id = msg.header.frame_id
        self.aruco_msg.header.stamp = rospy.Time.now()
        self.aruco_msg.child_frame_id = "marker_filtered"

        self.tf_br.sendTransform(self.aruco_msg)

if __name__ == '__main__':
    rospy.init_node('pose_filtering_node', anonymous=True)
    PoseKF()
    rospy.spin()