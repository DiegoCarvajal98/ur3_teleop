#! /usr/bin/env python
import rospy
import tf2_ros
import math
import numpy as np
from geometry_msgs.msg import TransformStamped, PoseStamped
from simple_kf import SimpleKalmanFilter

class PoseKF():
    def __init__(self):
        self.filt_pos_x = SimpleKalmanFilter(0.5,0.5,0.5)
        self.filt_pos_y = SimpleKalmanFilter(0.5,0.5,0.5)
        self.filt_pos_z = SimpleKalmanFilter(0.5,0.5,0.5)
        self.filt_ori_x = SimpleKalmanFilter(0.5,0.5,0.5)
        self.filt_ori_y = SimpleKalmanFilter(0.5,0.5,0.5)
        self.filt_ori_z = SimpleKalmanFilter(0.5,0.5,0.5)
        self.filt_ori_w = SimpleKalmanFilter(0.5,0.5,0.5)

        self.aruco_msg = TransformStamped()
        self.tf_br = tf2_ros.TransformBroadcaster()
        self.aruco_subscriber = rospy.Subscriber('/aruco_single/pose',PoseStamped,self.arucoCallback,queue_size=5)

    def arucoCallback(self,msg):
        self.aruco_msg.transform.translation.x = self.filt_pos_x.predict(msg.pose.position.x)
        self.aruco_msg.transform.translation.y = self.filt_pos_y.predict(msg.pose.position.y)
        self.aruco_msg.transform.translation.z = self.filt_pos_z.predict(msg.pose.position.z)
        x = self.filt_ori_x.predict(msg.pose.orientation.x)
        y = self.filt_ori_y.predict(msg.pose.orientation.y)
        z = self.filt_ori_z.predict(msg.pose.orientation.z)
        w = self.filt_ori_w.predict(msg.pose.orientation.w)

        q = np.array([x, y, z, w])

        q_norm = self.normalize(q)

        self.aruco_msg.transform.rotation.x = q_norm[0]
        self.aruco_msg.transform.rotation.y = q_norm[1]
        self.aruco_msg.transform.rotation.z = q_norm[2]
        self.aruco_msg.transform.rotation.w = q_norm[3]
        self.aruco_msg.header.frame_id = msg.header.frame_id
        self.aruco_msg.header.stamp = rospy.Time.now()
        self.aruco_msg.child_frame_id = "marker_filtered"

        try:
            self.tf_br.sendTransform(self.aruco_msg)
        except (tf2_ros.TransformException):
            pass

    def normalize(self, v, tolerance=0.00001):
        mag2 = sum(n * n for n in v)
        if mag2 > tolerance:
            mag = math.sqrt(mag2)
            v = tuple(n / mag for n in v)
        return np.array(v)

if __name__ == '__main__':
    rospy.init_node('pose_filtering_node', anonymous=True)
    PoseKF()
    rospy.spin()