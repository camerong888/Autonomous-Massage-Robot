#!/usr/bin/env python3

import pyrealsense2 as rs
import numpy as np
import cv2
from ultralytics import YOLO
import rospy
from geometry_msgs.msg import PoseStamped, Point
from std_msgs.msg import Header
from visualization_msgs.msg import Marker
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

class VideoStreamPublisher:
    def __init__(self):
        self.image_pub = rospy.Publisher('/camera/color/image_raw', Image, queue_size=10)
        self.bridge = CvBridge()  # OpenCV to ROS Image conversion

    def publish_image(self, color_image):
        image_msg = self.bridge.cv2_to_imgmsg(color_image, encoding="bgr8")
        self.image_pub.publish(image_msg)

class DepthDistancePublisher:
    def __init__(self):
        self.marker_pub = rospy.Publisher('/keypoint_distances', Marker, queue_size=10)

    def publish_distance_marker(self, x, y, z, joint_name, marker_id, frame_id="camera_link"):
        marker = Marker()
        marker.header.frame_id = frame_id
        marker.header.stamp = rospy.Time.now()
        marker.ns = "distance_markers"
        marker.id = marker_id
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD
        marker.scale.z = 0.1  # Text size
        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 1.0
        marker.color.a = 1.0
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = z
        marker.text = f"{joint_name}: {z:.2f}m"
        self.marker_pub.publish(marker)

class RealSenseHandler:
    def __init__(self):
        self.pipeline = rs.pipeline()
        self.config = rs.config()
        self.config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        self.config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        self.profile = self.pipeline.start(self.config)

        self.depth_scale = self.profile.get_device().first_depth_sensor().get_depth_scale()
        self.intrinsics = self.profile.get_stream(rs.stream.depth).as_video_stream_profile().get_intrinsics()
        self.align = rs.align(rs.stream.color)

    def get_aligned_frames(self):
        frames = self.pipeline.wait_for_frames()
        aligned_frames = self.align.process(frames)
        color_frame = aligned_frames.get_color_frame()
        depth_frame = aligned_frames.get_depth_frame()
        if not color_frame or not depth_frame:
            return None, None
        color_image = np.asanyarray(color_frame.get_data())
        depth_image = np.asanyarray(depth_frame.get_data())
        return color_image, depth_image

    def stop(self):
        self.pipeline.stop()

class YOLOPoseEstimator:
    def __init__(self, model_path):
        self.model = YOLO(model_path)
        self.arm_connections = [(5, 7), (7, 9), (6, 8), (8, 10)]  # Arm joint connections

    def estimate_pose(self, image):
        return self.model(image, stream=True)

class PosePublisher:
    def __init__(self):
        rospy.init_node('pose_publisher')
        self.pose_pub = rospy.Publisher('/pose_estimation', PoseStamped, queue_size=10)

    def publish_pose(self, x, y, z, frame_id="camera_link"):
        pose_msg = PoseStamped()
        pose_msg.header = Header(stamp=rospy.Time.now(), frame_id=frame_id)
        pose_msg.pose.position.x = x
        pose_msg.pose.position.y = y
        pose_msg.pose.position.z = z
        pose_msg.pose.orientation.w = 1.0
        self.pose_pub.publish(pose_msg)

class Arm3DVisualizer:
    def __init__(self, realsense_handler, pose_estimator, pose_publisher):
        self.realsense_handler = realsense_handler
        self.pose_estimator = pose_estimator
        self.pose_publisher = pose_publisher
        self.video_stream_publisher = VideoStreamPublisher()
        self.distance_publisher = DepthDistancePublisher()
        self.marker_pub = rospy.Publisher('/arm_pose_markers', Marker, queue_size=10)

    def publish_joint_marker(self, x, y, z, joint_name, marker_id, frame_id="camera_link"):
        joint_marker = Marker()
        joint_marker.header.frame_id = frame_id
        joint_marker.header.stamp = rospy.Time.now()
        joint_marker.ns = "arm_joints"
        joint_marker.id = marker_id
        joint_marker.type = Marker.SPHERE
        joint_marker.action = Marker.ADD
        joint_marker.pose.position.x = x
        joint_marker.pose.position.y = y
        joint_marker.pose.position.z = z
        joint_marker.pose.orientation.w = 1.0
        joint_marker.scale.x = 0.05
        joint_marker.scale.y = 0.05
        joint_marker.scale.z = 0.05
        joint_marker.color.r = 1.0
        joint_marker.color.g = 0.0
        joint_marker.color.b = 0.0
        joint_marker.color.a = 1.0
        self.marker_pub.publish(joint_marker)

    def publish_connection_marker(self, points_3d, frame_id="camera_link"):
        connection_marker = Marker()
        connection_marker.header.frame_id = frame_id
        connection_marker.header.stamp = rospy.Time.now()
        connection_marker.ns = "arm_connections"
        connection_marker.id = 0
        connection_marker.type = Marker.LINE_LIST
        connection_marker.action = Marker.ADD
        connection_marker.scale.x = 0.02
        connection_marker.color.r = 0.0
        connection_marker.color.g = 1.0
        connection_marker.color.b = 0.0
        connection_marker.color.a = 1.0
        connection_marker.points = []

        for (start_idx, end_idx) in self.pose_estimator.arm_connections:
            if start_idx < len(points_3d) and end_idx < len(points_3d):
                start_point = points_3d[start_idx]
                end_point = points_3d[end_idx]
                if start_point != (0, 0, 0) and end_point != (0, 0, 0):
                    connection_marker.points.append(Point(*start_point))
                    connection_marker.points.append(Point(*end_point))

        if connection_marker.points:  # Only publish if there are points to connect
            self.marker_pub.publish(connection_marker)

    def draw_pose_and_publish(self, color_image, depth_image):
        results = self.pose_estimator.estimate_pose(color_image)
        points_3d = []

        for result in results:
            boxes = result.boxes
            keypoints = result.keypoints

            for i, box in enumerate(boxes):
                x1, y1, x2, y2 = map(int, box.xyxy[0])
                cv2.rectangle(color_image, (x1, y1), (x2, y2), (0, 255, 0), 2)

                if keypoints is not None:
                    points = [(int(point[0]), int(point[1])) for point in keypoints.xy[i]]
                    for idx, (x, y) in enumerate(points):
                        if idx in [5, 6, 7, 8, 9, 10] and (0 <= x < depth_image.shape[1]) and (0 <= y < depth_image.shape[0]):
                            depth = depth_image[y, x] * self.realsense_handler.depth_scale
                            x3d, y3d, z3d = rs.rs2_deproject_pixel_to_point(self.realsense_handler.intrinsics, [x, y], depth)
                            points_3d.append((x3d, y3d, z3d))
                            self.pose_publisher.publish_pose(x3d, y3d, z3d)

                            joint_name = ["Left Shoulder", "Right Shoulder", "Left Elbow", "Right Elbow", "Left Wrist", "Right Wrist"][idx - 5]
                            self.publish_joint_marker(x3d, y3d, z3d, joint_name, marker_id=idx)
                            self.distance_publisher.publish_distance_marker(x3d, y3d, z3d, joint_name, marker_id=idx)

                            position_text = f"{joint_name}: ({x3d:.2f}m, {y3d:.2f}m, {z3d:.2f}m)"
                            cv2.putText(color_image, position_text, (x + 5, y - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)

                    self.publish_connection_marker(points_3d)
                    for (start_idx, end_idx) in self.pose_estimator.arm_connections:
                        if start_idx < len(points) and end_idx < len(points):
                            start_point, end_point = points[start_idx], points[end_idx]
                            if start_point != (0, 0) and end_point != (0, 0):
                                cv2.line(color_image, start_point, end_point, (255, 0, 0), 2)

        self.video_stream_publisher.publish_image(color_image)
        return color_image

def main():
    realsense_handler = RealSenseHandler()
    pose_estimator = YOLOPoseEstimator("models/yolo11n-pose.pt")
    pose_publisher = PosePublisher()
    visualizer = Arm3DVisualizer(realsense_handler, pose_estimator, pose_publisher)

    try:
        while not rospy.is_shutdown():
            color_image, depth_image = realsense_handler.get_aligned_frames()
            if color_image is None or depth_image is None:
                continue

            annotated_image = visualizer.draw_pose_and_publish(color_image, depth_image)
            cv2.imshow("RealSense Color Image with Pose Estimation", annotated_image)
            cv2.imshow("Depth Heat Map", cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET))

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
    finally:
        realsense_handler.stop()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
