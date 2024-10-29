#!/usr/bin/env python3
# impedance_controller.py

import rospy
from kortex_driver.srv import MoveToTwist, MoveToTwistRequest
from kortex_driver.msg import BaseCyclic_Feedback
from geometry_msgs.msg import Twist
from std_msgs.msg import String

class ImpedanceController:
    def __init__(self):
        rospy.init_node('impedance_controller', anonymous=True)
        
        # Service proxies
        self.move_to_twist = rospy.ServiceProxy('/my_gen3/base/move_to_twist', MoveToTwist)
        self.stop_arm = rospy.ServiceProxy('/my_gen3/base/stop', Stop)
        
        # Parameters for impedance control
        self.stiffness = rospy.get_param("~stiffness", 1.0)
        self.damping = rospy.get_param("~damping", 0.5)
        
        # Current feedback
        self.current_feedback = BaseCyclic_Feedback()
        rospy.Subscriber('/my_gen3/base_feedback', BaseCyclic_Feedback, self.feedback_callback)
        
        # Subscriber to desired Twist commands
        rospy.Subscriber('/desired_twist', Twist, self.twist_callback)
        
        rospy.loginfo("Impedance Controller Initialized.")

    def feedback_callback(self, msg):
        self.current_feedback = msg

    def twist_callback(self, desired_twist):
        # Simple impedance control: controlled_twist = stiffness * desired_twist - damping * current_velocity
        controlled_twist = Twist()
        controlled_twist.linear.x = self.stiffness * desired_twist.linear.x - self.damping * self.current_feedback.twist.linear.x
        controlled_twist.linear.y = self.stiffness * desired_twist.linear.y - self.damping * self.current_feedback.twist.linear.y
        controlled_twist.linear.z = self.stiffness * desired_twist.linear.z - self.damping * self.current_feedback.twist.linear.z
        controlled_twist.angular.x = self.stiffness * desired_twist.angular.x - self.damping * self.current_feedback.twist.angular.x
        controlled_twist.angular.y = self.stiffness * desired_twist.angular.y - self.damping * self.current_feedback.twist.angular.y
        controlled_twist.angular.z = self.stiffness * desired_twist.angular.z - self.damping * self.current_feedback.twist.angular.z
        
        request = MoveToTwistRequest()
        request.twist = controlled_twist
        request.reference_frame_id = "base_link"
        request.min_duration = rospy.Duration(0.1)
        request.max_duration = rospy.Duration(1.0)
        
        try:
            response = self.move_to_twist(request)
            rospy.loginfo(f"MoveToTwist Response: {response.result}")
        except rospy.ServiceException as e:
            rospy.logerr(f"MoveToTwist Service call failed: {e}")

    def stop_massage(self):
        try:
            response = self.stop_arm()
            rospy.loginfo(f"Stop Arm Response: {response.result}")
        except rospy.ServiceException as e:
            rospy.logerr(f"Stop Service call failed: {e}")

    def run(self):
        rospy.spin()

if __name__ == "__main__":
    controller = ImpedanceController()
    try:
        controller.run()
    except rospy.ROSInterruptException:
        controller.stop_massage()
