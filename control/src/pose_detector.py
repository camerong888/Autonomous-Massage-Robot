import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from sensor_msgs.msg import Image
import cv2
import cv_bridge
from ultralytics import YOLO


class PoseDetector:
    def __init__(self):

        self.bridge = cv_bridge.CvBridge()
        self.yolo_model = YOLO('/home/ar/Downloads/yolo11m-pose.pt')
        self.image_sub = rospy.Subscriber("/camera/color/image_raw", Image, self.image_callback)
        self.pose_pub = rospy.Publisher("/detected_poses", String, queue_size=10)
        self.detected_pose_pub = rospy.Publisher("/detected_pose", Image, queue_size=10)

    def image_callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except cv_bridge.CvBridgeError as e:
            rospy.logerr(e)
            return

        results = self.yolo_model(cv_image)

        print(results)
        for result in results:
            for keypoint in result.keypoints:
                print("KEYPOINT", keypoint)
                for xy in keypoint.xy[0]:
                    print("XY", xy)
                    cv2.circle(cv_image, (int(xy[0]), int(xy[1])), 5, (0, 255, 0), -1)

        self.detected_pose_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
if __name__ == '__main__':
    rospy.init_node('pose_detector', anonymous=True)
    pd = PoseDetector()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("Shutting down")