#!/usr/bin/env python3

import rospy
import tf
import tf2_ros
import tf2_geometry_msgs
from apriltag_ros.msg import AprilTagDetectionArray
from geometry_msgs.msg import PoseStamped
import yaml
from tf2_ros import TransformStamped

#the separate node to lookup base_link to camera_link transformation and broadcast it

class RealsenseTransformBroadcaster:
    def __init__(self):
        rospy.init_node("realsense_tf_broadcaster", anonymous=True)

        self.tf_buffer= tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.broadcaster = tf2_ros.TransformBroadcaster()

        rospy.sleep(1)
        self.transform = self.tf_buffer.lookup_transform("base_link", "realsense_link", rospy.Time(0))

        self.timer = rospy.Timer(rospy.Duration(0.1), self.timer_callback)
    
    def timer_callback(self, event):
        try:
            self.transform.header.stamp = rospy.Time.now()
            self.broadcaster.sendTransform(self.transform)
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rospy.logwarn(f"Transform lookup failed for Realsense camera:{e}")

if __name__ == "__main__":
    processor = RealsenseTransformBroadcaster()
    rospy.spin()

