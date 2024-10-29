#!/usr/bin/env python3
# arm_controller.py

import rospy
from kortex_driver.srv import MoveToPose, MoveToPoseRequest
from geometry_msgs.msg import Pose

class ArmController:
    def __init__(self):
        rospy.init_node('arm_controller', anonymous=True)
        
        # Service proxies
        self.move_to_pose = rospy.ServiceProxy('/my_gen3/base/move_to_pose', MoveToPose)
        self.stop_arm = rospy.ServiceProxy('/my_gen3/base/stop', Stop)
        
        # Subscriber to desired poses
        rospy.Subscriber('/desired_pose', Pose, self.handle_desired_pose)
        
        rospy.loginfo("Arm Controller Initialized.")

    def handle_desired_pose(self, pose_msg):
        request = MoveToPoseRequest()
        request.pose = pose_msg
        request.reference_frame_id = "base_link"
        request.min_duration = rospy.Duration(2.0)
        request.max_duration = rospy.Duration(5.0)
        
        try:
            response = self.move_to_pose(request)
            rospy.loginfo(f"MoveToPose Response: {response.result}")
        except rospy.ServiceException as e:
            rospy.logerr(f"MoveToPose Service call failed: {e}")

    def stop_massage(self):
        try:
            response = self.stop_arm()
            rospy.loginfo(f"Stop Arm Response: {response.result}")
        except rospy.ServiceException as e:
            rospy.logerr(f"Stop Service call failed: {e}")

    def run(self):
        rospy.spin()

if __name__ == "__main__":
    controller = ArmController()
    try:
        controller.run()
    except rospy.ROSInterruptException:
        controller.stop_massage()
