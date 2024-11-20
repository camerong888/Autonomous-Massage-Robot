#!/usr/bin/env python3

#original test1 :)

import pyrealsense2 as rs
import numpy as np
import cv2
from pupil_apriltags import Detector
from ultralytics import YOLO
import rospy
from geometry_msgs.msg import PoseStamped, Point
from std_msgs.msg import Header
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CameraInfo
import tf
from visualization_msgs.msg import Marker
from tf import transformations as t

class VisionPoseAprilTag:
    def __init__(self):
        rospy.init_node('vision_pose_apriltag', anonymous=True)
        self.image_pub = rospy.Publisher('/realsense/color/image_raw', Image, queue_size=10)
        self.pose_pub = rospy.Publisher('/pose_estimation', PoseStamped, queue_size=10)
        self.marker_pub = rospy.Publisher('/human_pose_markers', Marker, queue_size=10)
        self.camera_info_pub = rospy.Publisher('/realsense/color/camera_info', CameraInfo, queue_size=10)

        self.bridge = CvBridge()
        self.model = YOLO("models/yolo11n-pose.pt")
        self.arm_connections = [(5, 7), (7, 9), (6, 8), (8, 10)]
        self.at_detector = Detector(families='tag36h11')

        # Initialize RealSense
        self.pipeline = rs.pipeline()
        config = rs.config()
        config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        profile = self.pipeline.start(config)
        self.depth_scale = profile.get_device().first_depth_sensor().get_depth_scale()
        self.intrinsics = profile.get_stream(rs.stream.depth).as_video_stream_profile().get_intrinsics()
        self.align = rs.align(rs.stream.color)

        # Get color camera intrinsics and map to CameraInfo message
        color_profile = profile.get_stream(rs.stream.color)
        color_intrinsics = color_profile.as_video_stream_profile().get_intrinsics()
        
        self.camera_info_msg = CameraInfo()
        self.camera_info_msg.width = color_intrinsics.width
        self.camera_info_msg.height = color_intrinsics.height
        self.camera_info_msg.K = [color_intrinsics.fx, 0, color_intrinsics.ppx, 0, color_intrinsics.fy, color_intrinsics.ppy, 0, 0, 1]
        self.camera_info_msg.P = [color_intrinsics.fx, 0, color_intrinsics.ppx, 0, 0, color_intrinsics.fy, color_intrinsics.ppy, 0, 0, 0, 1, 0]
        self.camera_info_msg.D = [0, 0, 0, 0, 0]  # Assuming no distortion
        self.camera_info_msg.distortion_model = "plumb_bob"
        self.camera_info_msg.header = Header()
        self.camera_info_msg.header.frame_id = "realsense_link"

        # Initialize TF Broadcaster
        self.tf_broadcaster = tf.TransformBroadcaster()

    def get_aligned_frames(self):
        frames = self.pipeline.wait_for_frames()
        aligned_frames = self.align.process(frames)
        color_frame = aligned_frames.get_color_frame()
        depth_frame = aligned_frames.get_depth_frame()
        if not color_frame or not depth_frame:
            return None, None
        return np.asanyarray(color_frame.get_data()), np.asanyarray(depth_frame.get_data())

    def detect_apriltags(self, color_image):
        gray = cv2.cvtColor(color_image, cv2.COLOR_BGR2GRAY)
        tags = self.at_detector.detect(
            gray, estimate_tag_pose=True,
            camera_params=[self.intrinsics.fx, self.intrinsics.fy, self.intrinsics.ppx, self.intrinsics.ppy],
            tag_size=0.14
        )
        for tag in tags:
            self.publish_apriltag_tf(tag)  # Broadcast the TF for each tag
            for (p1, p2) in zip(tag.corners, np.roll(tag.corners, -1, axis=0)):
                cv2.line(color_image, tuple(map(int, p1)), tuple(map(int, p2)), (0, 255, 0), 2)
            cX, cY = map(int, tag.center)
            cv2.circle(color_image, (cX, cY), 5, (0, 0, 255), -1)
            cv2.putText(color_image, f"ID: {tag.tag_id}", (cX, cY - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 2)

    def publish_apriltag_tf(self, tag):
        translation = tag.pose_t.flatten()
        rotation = tag.pose_R

        # Construct 4x4 transformation matrix
        T = np.eye(4)
        T[:3, :3] = rotation
        T[:3, 3] = translation
        print(T)

        # Convert to quaternion
        quaternion = tf.transformations.quaternion_from_matrix(T)

        inverse_tf = t.inverse_matrix(T)
        print(inverse_tf)

        # Broadcast Inverse TF
        inverse_translation = t.translation_from_matrix(inverse_tf)
        inverse_quaternion = t.quaternion_from_matrix(inverse_tf)

        inverse_translation = -1 * inverse_translation

        self.tf_broadcaster.sendTransform(
            inverse_translation,
            inverse_quaternion,
            rospy.Time.now(),
            "realsense_link",
            f"apriltag_{tag.tag_id}"
        )
        
        # Broadcast TF
        # self.tf_broadcaster.sendTransform(
            # translation,
            # quaternion,
            # rospy.Time.now(),
            # "realsense_link",
            # f"apriltag_{tag.tag_id}"
        # )

    def detect_pose(self, color_image, depth_image):
        results = self.model(color_image, stream=True)
        points_3d = []
        for result in results:
            if result.keypoints is None:
                continue
            keypoints = result.keypoints.xy[0]
            for idx, (x, y) in enumerate(keypoints):
                if idx not in [5, 6, 7, 8, 9, 10]:  # Only arms
                    continue
                x, y = int(x), int(y)
                if 0 <= x < depth_image.shape[1] and 0 <= y < depth_image.shape[0]:
                    depth = depth_image[y, x] * self.depth_scale
                    x3d, y3d, z3d = rs.rs2_deproject_pixel_to_point(self.intrinsics, [x, y], depth)
                    points_3d.append((x3d, y3d, z3d))
                    cv2.circle(color_image, (x, y), 3, (255, 0, 0), -1)
        self.publish_joint_markers(points_3d)
        self.publish_joint_tfs(points_3d)

    def publish_joint_markers(self, points_3d):
        marker = Marker()
        marker.header.frame_id = "realsense_link"
        marker.header.stamp = rospy.Time.now()
        marker.ns = "human_pose"
        marker.id = 0
        marker.type = Marker.SPHERE_LIST
        marker.action = Marker.ADD
        marker.scale.x = 0.05
        marker.scale.y = 0.05
        marker.scale.z = 0.05
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 1.0

        for point in points_3d:
            if point != (0, 0, 0):
                marker.points.append(Point(*point))

        self.marker_pub.publish(marker)

    def publish_joint_tfs(self, points_3d):
        for idx, point in enumerate(points_3d):
            if point != (0, 0, 0):
                translation = point
                quaternion = [0, 0, 0, 1]
                self.tf_broadcaster.sendTransform(
                    translation,
                    quaternion,
                    rospy.Time.now(),
                    f"joint_{idx}",
                    "realsense_link"
                )

    def publish_image(self, image):
        image_msg = self.bridge.cv2_to_imgmsg(image, encoding="bgr8")
        stamp = rospy.Time.now()
        image_msg.header.stamp = stamp
        image_msg.header.frame_id = "realsense_link"
        self.image_pub.publish(image_msg)

        self.camera_info_msg.header.stamp = stamp
        self.camera_info_pub.publish(self.camera_info_msg)


    def run(self):
        try:
            while not rospy.is_shutdown():
                color_image, depth_image = self.get_aligned_frames()
                if color_image is None or depth_image is None:
                    continue
                # self.detect_apriltags(color_image)
                self.detect_pose(color_image, depth_image)

                cv2.imshow("RealSense: Pose and AprilTags", color_image)
                self.publish_image(color_image)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break
        finally:
            self.pipeline.stop()
            cv2.destroyAllWindows()


if __name__ == "__main__":
    VisionPoseAprilTag().run()
