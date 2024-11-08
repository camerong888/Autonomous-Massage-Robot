#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Image
import cv2
import cv_bridge
from ultralytics import YOLO
import pyrealsense2 as rs
import numpy as np
import cv2
from geometry_msgs.msg import PoseStamped, Point
from std_msgs.msg import Header
from visualization_msgs.msg import Marker
import message_filters

class PoseDetector:
    def __init__(self):

        self.bridge = cv_bridge.CvBridge()
        self.yolo_model = YOLO('/home/ar/Downloads/yolo11m-pose.pt')
        # self.image_sub = rospy.Subscriber("/camera/color/image_rect_color", Image, self.image_callback)
        # self.depth_sub = rospy.Subscriber("/camera/depth_registered/sw_registered/image_rect", Image, self.depth_callback)

        self.camera_matrix = [[360.01333, 0.00000, 243.87228],
                            [0.00000, 360.013366699, 137.9218444], 
                            [0.00000, 0.00000, 1.00000]]

        self.image_sub = message_filters.Subscriber("/camera/color/image_rect_color", Image)
        self.depth_sub = message_filters.Subscriber("/camera/depth_registered/sw_registered/image_rect", Image)
        ts = message_filters.ApproximateTimeSynchronizer([self.image_sub, self.depth_sub], 10, 0.2)
        ts.registerCallback(self.draw_pose_and_publish)
        print("Created synchronizer")

        self.pose_pub = rospy.Publisher("/detected_poses", String, queue_size=10)
        self.detected_pose_pub = rospy.Publisher("/detected_pose", Image, queue_size=10)
        self.marker_pub = rospy.Publisher('/keypoint_distances', Marker, queue_size=10)
        self.pose_pub = rospy.Publisher('/pose_estimation', PoseStamped, queue_size=10)
        self.marker_pub = rospy.Publisher('/arm_pose_markers', Marker, queue_size=10)
        self.arm_connections = [(5, 7), (7, 9), (6, 8), (8, 10)]  # Arm joint connections

    def depth_callback(self, data):
        try:
            depth_image = self.bridge.imgmsg_to_cv2(data, "16UC1")
        except cv_bridge.CvBridgeError as e:
            rospy.logerr(e)
            return
        
        print("DEPTH IMAGE", len(depth_image), len(depth_image[0]))

        depth_image = np.array(depth_image, dtype=np.uint16)
        self.depth_image = depth_image

    def image_callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except cv_bridge.CvBridgeError as e:
            rospy.logerr(e)
            return
        print("CV IMAGE", len(cv_image), len(cv_image[0]))
        results = self.yolo_model(cv_image)

        # print(results)
        for result in results:
            for keypoint in result.keypoints:
                # print("KEYPOINT", keypoint)
                for xy in keypoint.xy[0]:
                    # print("XY", xy)
                    cv2.circle(cv_image, (int(xy[0]), int(xy[1])), 5, (0, 255, 0), -1)

        self.detected_pose_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))

    def publish_image(self, color_image):
        image_msg = self.bridge.cv2_to_imgmsg(color_image, encoding="bgr8")
        self.detected_pose_pub.publish(image_msg)



    def publish_pose(self, x, y, z, frame_id="camera_link"):
        pose_msg = PoseStamped()
        pose_msg.header = Header(stamp=rospy.Time.now(), frame_id=frame_id)
        pose_msg.pose.position.x = x
        pose_msg.pose.position.y = y
        pose_msg.pose.position.z = z
        pose_msg.pose.orientation.w = 1.0
        self.pose_pub.publish(pose_msg)



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

        for (start_idx, end_idx) in self.arm_connections:
            if start_idx < len(points_3d) and end_idx < len(points_3d):
                start_point = points_3d[start_idx]
                end_point = points_3d[end_idx]
                if start_point != (0, 0, 0) and end_point != (0, 0, 0):
                    connection_marker.points.append(Point(*start_point))
                    connection_marker.points.append(Point(*end_point))

        if connection_marker.points:  # Only publish if there are points to connect
            self.marker_pub.publish(connection_marker)

    def draw_pose_and_publish(self, color_image_msg, depth_image_msg):
        try:
            color_image = self.bridge.imgmsg_to_cv2(color_image_msg, "bgr8")
            depth_image = self.bridge.imgmsg_to_cv2(depth_image_msg, "16UC1")
        except cv_bridge.CvBridgeError as e:
            rospy.logerr(e)
            return
        print("DRAWING POSE")
        results = self.yolo_model(color_image)
        points_3d = []
        print(results)

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


                            # depth = depth_image[y, x] * self.realsense_handler.depth_scale
                            depth = depth_image[y, x]
                            x3d = (x - self.camera_matrix[0][2]) * depth / self.camera_matrix[0][0]
                            y3d = (y - self.camera_matrix[1][2]) * depth / self.camera_matrix[1][1]
                            z3d = depth

                            # x3d, y3d, z3d = rs.rs2_deproject_pixel_to_point(self.realsense_handler.intrinsics, [x, y], depth)


                            points_3d.append((x3d, y3d, z3d))
                            self.publish_pose(x3d, y3d, z3d)

                            joint_name = ["Left Shoulder", "Right Shoulder", "Left Elbow", "Right Elbow", "Left Wrist", "Right Wrist"][idx - 5]
                            self.publish_joint_marker(x3d, y3d, z3d, joint_name, marker_id=idx)
                            self.publish_distance_marker(x3d, y3d, z3d, joint_name, marker_id=idx)

                            position_text = f"{joint_name}: ({x3d:.2f}m, {y3d:.2f}m, {z3d:.2f}m)"
                            cv2.putText(color_image, position_text, (x + 5, y - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)

                    self.publish_connection_marker(points_3d)
                    for (start_idx, end_idx) in self.arm_connections:
                        if start_idx < len(points) and end_idx < len(points):
                            start_point, end_point = points[start_idx], points[end_idx]
                            if start_point != (0, 0) and end_point != (0, 0):
                                cv2.line(color_image, start_point, end_point, (255, 0, 0), 2)

        self.detected_pose_pub.publish(self.bridge.cv2_to_imgmsg(color_image, "bgr8"))
        return color_image


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



def main():
    realsense_handler = RealSenseHandler()
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


if __name__ == '__main__':
    rospy.init_node('pose_detector', anonymous=True)
    pd = PoseDetector()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("Shutting down")