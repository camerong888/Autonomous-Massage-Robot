#!/usr/bin/env python3

import rospy
import tf
import tf2_ros
import tf2_geometry_msgs
from apriltag_ros.msg import AprilTagDetectionArray
from geometry_msgs.msg import PoseStamped
import yaml
from tf2_ros import TransformStamped

class AprilTagProcessor:
    # initialize ros parameters as apriltag_tf_processor and buffer for tf2 transform
    def __init__(self):
        rospy.init_node("apriltag_tf_processor", anonymous=True)

        self.arm_apriltag_name = "apriltag_arm"
        self.realsense_apriltag_name = "apriltag_realsense"

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.broadcaster = tf2_ros.TransformBroadcaster()
        self.static_broadcaster = tf2_ros.StaticTransformBroadcaster()

        # Create a timer that calls the callback function every second
        self.timer = rospy.Timer(rospy.Duration(0.1), self.timer_callback)
    
    
    def timer_callback(self, event):
        self.align_apriltag_frames()
    

    def align_apriltag_frames(self):
        """
        put `apriltag_arm` and `apriltag_realsense` frame aligned to 0 
        """
        
        try:
            # # get `apriltag_arm` to `apriltag_realsense` transformation
            # transform = self.tf_buffer.lookup_transform(
            #     "apriltag_realsense", "apriltag_arm", rospy.Time(0)
            # )

            # # publish the static transformation and align both to (0, 0, 0)
            # self.broadcast_static_transform(
            #     parent_frame="apriltag_realsense",
            #     child_frame="aligned_apriltag_frame",
            #     translation=[0.0, 0.0, 0.0],
            #     rotation=[0.0, 0.0, 0.0, 1.0]  # unit quaternion    
    
        
            transform_to_arm = self.tf_buffer.lookup_transform("apriltag_realsense_0", "realsense_link", rospy.Time(0))
            self.broadcast_static_transform(
                parent_frame="apriltag_arm_0",
                child_frame="realsense_link",
                translation=[
                    transform_to_arm.transform.translation.x,
                    transform_to_arm.transform.translation.y,
                    transform_to_arm.transform.translation.z,
                ],
                rotation=[
                    transform_to_arm.transform.rotation.x,
                    transform_to_arm.transform.rotation.y,
                    transform_to_arm.transform.rotation.z,
                    transform_to_arm.transform.rotation.w,
                ],
            )

            # self.broadcast_static_transform(
            #     parent_frame="apriltag_realsense_0",
            #     child_frame="aligned_apriltag_frame",
            #     translation=[0.0,0.0,0.0],
            #     rotation=[0.0,0.0,0.0,1.0],
            # )

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            rospy.logwarn(f"Transform lookup failed for aligning AprilTag frames. {e}")
        
        
        # except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
        #     rospy.logwarn("Transform lookup failed for aligning AprilTag frames.")

    def broadcast_realsense_depth_camera(self):
        """
        broadcast Realsense Depth Camera static transformation
        """


        """
        parent_frame="apriltag_realsense_0",
        child_frame="realsense_depth_camera",
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
