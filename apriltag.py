#!/usr/bin/env python3

import rospy
import tf
import tf2_ros
import tf2_geometry_msgs
from apriltag_ros.msg import AprilTagDetectionArray
from geometry_msgs.msg import PoseStamped
import yaml


# loading the parameters of kinova camera
def load_camera_parameters():
    camera_params = {
        "fx": 1297.672904,
        "fy": 1298.631344,
        "cx": 620.914026,
        "cy": 238.280325,
        "distortion": [0.0, 0.0, 0.0, 0.0, 0.0]
    }
    return camera_params

class AprilTagProcessor:
    # initialize ros parameters as apriltag_tf_processor and buffer for tf2 transform
    def __init__(self):
        rospy.init_node("apriltag_tf_processor", anonymous=True)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.broadcaster = tf2_ros.TransformBroadcaster()
        self.static_broadcaster = tf2_ros.StaticTransformBroadcaster()

        # subscribe AprilTag detection
        rospy.Subscriber("/tag_detections", AprilTagDetectionArray, self.apriltag_callback)

    def apriltag_callback(self, data):
        """
        AprilTag detection and update TF
        """
        if len(data.detections) > 0:
            # The first detected as the goal
            tag = data.detections[0]
            tag_pose = tag.pose.pose.pose

            # publish `apriltag_arm` frame
            self.broadcast_transform(
                parent_frame="arm_camera_frame",
                child_frame="apriltag_arm",
                pose=tag_pose
            )

            # publish `apriltag_realsense` frame
            self.broadcast_transform(
                parent_frame="realsense_camera_frame",
                child_frame="apriltag_realsense",
                pose=tag_pose
            )

            # align frames
            self.align_apriltag_frames()

            # broadcase Realsense Depth Camera static transform
            self.broadcast_realsense_depth_camera()

    def broadcast_transform(self, parent_frame, child_frame, pose):
        """
        publish Transform
        """
        transform = TransformStamped()
        transform.header.stamp = rospy.Time.now()
        transform.header.frame_id = parent_frame
        transform.child_frame_id = child_frame
        transform.transform.translation.x = pose.position.x
        transform.transform.translation.y = pose.position.y
        transform.transform.translation.z = pose.position.z
        transform.transform.rotation = pose.orientation

        self.broadcaster.sendTransform(transform)

    def align_apriltag_frames(self):
        """
        put `apriltag_arm` and `apriltag_realsense` frame aligned to 0 
        """
        try:
            # get `apriltag_arm` to `apriltag_realsense` transformation
            transform = self.tf_buffer.lookup_transform(
                "apriltag_realsense", "apriltag_arm", rospy.Time(0)
            )

            # publish the static transformation and align both to (0, 0, 0)
            self.broadcast_static_transform(
                parent_frame="apriltag_realsense",
                child_frame="aligned_apriltag_frame",
                translation=[0.0, 0.0, 0.0],
                rotation=[0.0, 0.0, 0.0, 1.0]  # 单位四元数
            )
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rospy.logwarn("Transform lookup failed for aligning AprilTag frames.")

    def broadcast_realsense_depth_camera(self):
        """
        broadcast Realsense Depth Camera static transformation
        """
        self.broadcast_static_transform(
            parent_frame="aligned_apriltag_frame",
            child_frame="realsense_depth_camera",
            translation=[0.1, 0.0, 0.3],  # example translation
            rotation=tf.transformations.quaternion_from_euler(0, 0, 0)
        )

    def broadcast_static_transform(self, parent_frame, child_frame, translation, rotation):
        """
        publish static Transform
        """
        static_transform = TransformStamped()
        static_transform.header.stamp = rospy.Time.now()
        static_transform.header.frame_id = parent_frame
        static_transform.child_frame_id = child_frame
        static_transform.transform.translation.x = translation[0]
        static_transform.transform.translation.y = translation[1]
        static_transform.transform.translation.z = translation[2]
        static_transform.transform.rotation.x = rotation[0]
        static_transform.transform.rotation.y = rotation[1]
        static_transform.transform.rotation.z = rotation[2]
        static_transform.transform.rotation.w = rotation[3]

        self.static_broadcaster.sendTransform(static_transform)


if __name__ == "__main__":
    processor = AprilTagProcessor()
    rospy.spin()