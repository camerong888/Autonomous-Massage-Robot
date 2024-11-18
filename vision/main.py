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

# Publishers for image, markers, and poses
image_pub = rospy.Publisher('/camera/color/image_raw', Image, queue_size=10)
pose_pub = rospy.Publisher('/pose_estimation', PoseStamped, queue_size=10)
distance_marker_pub = rospy.Publisher('/keypoint_distances', Marker, queue_size=10)
connection_marker_pub = rospy.Publisher('/arm_pose_markers', Marker, queue_size=10)

# OpenCV bridge for ROS Image conversion
bridge = CvBridge()

# YOLO model and arm connections
model = YOLO("models/yolo11n-pose.pt")
arm_connections = [(5, 7), (7, 9), (6, 8), (8, 10)]  # Arm joint connections

# RealSense setup
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
profile = pipeline.start(config)
depth_scale = profile.get_device().first_depth_sensor().get_depth_scale()
intrinsics = profile.get_stream(rs.stream.depth).as_video_stream_profile().get_intrinsics()
align = rs.align(rs.stream.color)

def get_aligned_frames():
    frames = pipeline.wait_for_frames()
    aligned_frames = align.process(frames)
    color_frame = aligned_frames.get_color_frame()
    depth_frame = aligned_frames.get_depth_frame()
    if not color_frame or not depth_frame:
        return None, None
    color_image = np.asanyarray(color_frame.get_data())
    depth_image = np.asanyarray(depth_frame.get_data())
    return color_image, depth_image

def publish_image(color_image):
    image_msg = bridge.cv2_to_imgmsg(color_image, encoding="bgr8")
    image_pub.publish(image_msg)

def publish_pose(x, y, z, frame_id="camera_link"):
    pose_msg = PoseStamped()
    pose_msg.header = Header(stamp=rospy.Time.now(), frame_id=frame_id)
    pose_msg.pose.position.x = x
    pose_msg.pose.position.y = y
    pose_msg.pose.position.z = z
    pose_msg.pose.orientation.w = 1.0
    pose_pub.publish(pose_msg)

def publish_distance_marker(x, y, z, joint_name, marker_id, frame_id="camera_link"):
    marker = Marker()
    marker.header.frame_id = frame_id
    marker.header.stamp = rospy.Time.now()
    marker.ns = "distance_markers"
    marker.id = marker_id
    marker.type = Marker.TEXT_VIEW_FACING
    marker.action = Marker.ADD
    marker.scale.z = 0.1
    marker.color.r = 1.0
    marker.color.g = 1.0
    marker.color.b = 1.0
    marker.color.a = 1.0
    marker.pose.position.x = x
    marker.pose.position.y = y
    marker.pose.position.z = z
    marker.text = f"{joint_name}: {z:.2f}m"
    distance_marker_pub.publish(marker)

def publish_joint_marker(x, y, z, joint_name, marker_id, frame_id="camera_link"):
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
    connection_marker_pub.publish(joint_marker)

def publish_connection_marker(points_3d, frame_id="camera_link"):
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

    for (start_idx, end_idx) in arm_connections:
        if start_idx < len(points_3d) and end_idx < len(points_3d):
            start_point = points_3d[start_idx]
            end_point = points_3d[end_idx]
            if start_point != (0, 0, 0) and end_point != (0, 0, 0):
                connection_marker.points.append(Point(*start_point))
                connection_marker.points.append(Point(*end_point))

    if connection_marker.points:  # Only publish if there are points to connect
        connection_marker_pub.publish(connection_marker)

def draw_pose_and_publish(color_image, depth_image):
    results = model(color_image, stream=True)
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
                        depth = depth_image[y, x] * depth_scale
                        x3d, y3d, z3d = rs.rs2_deproject_pixel_to_point(intrinsics, [x, y], depth)
                        points_3d.append((x3d, y3d, z3d))
                        publish_pose(x3d, y3d, z3d)

                        joint_name = ["Left Shoulder", "Right Shoulder", "Left Elbow", "Right Elbow", "Left Wrist", "Right Wrist"][idx - 5]
                        publish_joint_marker(x3d, y3d, z3d, joint_name, marker_id=idx)
                        publish_distance_marker(x3d, y3d, z3d, joint_name, marker_id=idx)

                        position_text = f"{joint_name}: ({x3d:.2f}m, {y3d:.2f}m, {z3d:.2f}m)"
                        cv2.putText(color_image, position_text, (x + 5, y - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)

                publish_connection_marker(points_3d)
                for (start_idx, end_idx) in arm_connections:
                    if start_idx < len(points) and end_idx < len(points):
                        start_point, end_point = points[start_idx], points[end_idx]
                        if start_point != (0, 0) and end_point != (0, 0):
                            cv2.line(color_image, start_point, end_point, (255, 0, 0), 2)

    publish_image(color_image)
    return color_image

def main():
    rospy.init_node('vision_pose_publisher', anonymous=True)
    try:
        while not rospy.is_shutdown():
            color_image, depth_image = get_aligned_frames()
            if color_image is None or depth_image is None:
                continue

            annotated_image = draw_pose_and_publish(color_image, depth_image)
            cv2.imshow("RealSense Color Image with Pose Estimation", annotated_image)
            cv2.imshow("Depth Heat Map", cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET))

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
    finally:
        pipeline.stop()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    main()

