#!/usr/bin/env python3

import rospy
import tf2_ros
import tf2_geometry_msgs
from apriltag_ros.msg import AprilTagDetectionArray

class AprilTagHandler:
    def __init__(self, base_frame):
        self.base_frame = base_frame
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        # Subscribe to AprilTag detections
        rospy.Subscriber('/tag_detections_kinova', AprilTagDetectionArray, self.kinova_callback)
        rospy.Subscriber('/tag_detections_depth', AprilTagDetectionArray, self.depth_callback)

    def kinova_callback(self, data):
        if len(data.detections) > 0:
            tag = data.detections[0]
            x, y, z = tag.pose.pose.pose.position.x, tag.pose.pose.pose.position.y, tag.pose.pose.pose.position.z
            self.transform_to_base_frame(x, y, z, 'kinova_camera_frame')

    def depth_callback(self, data):
        if len(data.detections) > 0:
            tag = data.detections[0]
            x, y, z = tag.pose.pose.pose.position.x, tag.pose.pose.pose.position.y, tag.pose.pose.pose.position.z
            self.transform_to_base_frame(x, y, z, 'depth_camera_frame')

    def transform_to_base_frame(self, x, y, z, source_frame):
        try:
            transform = self.tf_buffer.lookup_transform(self.base_frame, source_frame, rospy.Time(0))
            pose = tf2_geometry_msgs.PoseStamped()
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.position.z = z
            pose.pose.orientation.w = 1.0

            transformed_pose = tf2_geometry_msgs.do_transform_pose(pose, transform)
            rospy.loginfo(f"Transformed AprilTag position: x={transformed_pose.pose.position.x}, "
                          f"y={transformed_pose.pose.position.y}, z={transformed_pose.pose.position.z}")
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rospy.logwarn("Transform lookup failed.")

# Initialize the ROS node and AprilTagHandler
if __name__ == "__main__":
    rospy.init_node('apriltag_handler_node', anonymous=True)
    base_frame = 'base_frame'
    apriltag_handler = AprilTagHandler(base_frame)
    rospy.spin()
