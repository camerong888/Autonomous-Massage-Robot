import rospy
from apriltag_ros.msg import AprilTagDetectionArray

def apriltag_kinova_callback(data):
    if len(data.detections) > 0:
        tag = data.detections[0]
        kinova_x = tag.pose.pose.pose.position.x
        kinova_y = tag.pose.pose.pose.position.y
        kinova_z = tag.pose.pose.pose.position.z
        rospy.loginfo(f"Kinova Camera - AprilTag position: x={kinova_x}, y={kinova_y}, z={kinova_z}")
    else:
        rospy.loginfo("No AprilTag detected by Kinova Camera.")

def apriltag_depth_callback(data):
    if len(data.detections) > 0:
        tag = data.detections[0]
        depth_x = tag.pose.pose.pose.position.x
        depth_y = tag.pose.pose.pose.position.y
        depth_z = tag.pose.pose.pose.position.z
        rospy.loginfo(f"Depth Camera - AprilTag position: x={depth_x}, y={depth_y}, z={depth_z}")
    else:
        rospy.loginfo("No AprilTag detected by Depth Camera.")

rospy.init_node('apriltag_matcher', anonymous=True)
rospy.Subscriber('/tag_detections_kinova', AprilTagDetectionArray, apriltag_kinova_callback)
rospy.Subscriber('/tag_detections_depth', AprilTagDetectionArray, apriltag_depth_callback)
rospy.spin()
