#!/usr/bin/env python3
# path_planner.py

import rospy
from geometry_msgs.msg import Pose, Twist
import time

class PathPlanner:
    def __init__(self):
        rospy.init_node('path_planner', anonymous=True)
        
        # Publisher for desired poses
        self.pose_pub = rospy.Publisher('/desired_pose', Pose, queue_size=10)
        # Publisher for desired twists (if using impedance control)
        self.twist_pub = rospy.Publisher('/desired_twist', Twist, queue_size=10)
        
        # Example path: list of poses or twists
        self.path = self.generate_massage_path()
        self.current_step = 0
        self.rate = rospy.Rate(1)  # 1 Hz

    def generate_massage_path(self):
        # Placeholder for actual path planning logic
        # Return a list of Pose or Twist messages
        path = []
        pose = Pose()
        pose.position.x = 0.5
        pose.position.y = 0.0
        pose.position.z = 0.5
        pose.orientation.w = 1.0
        path.append(pose)
        
        pose2 = Pose()
        pose2.position.x = 0.6
        pose2.position.y = 0.1
        pose2.position.z = 0.5
        pose2.orientation.w = 1.0
        path.append(pose2)
        
        # Add more poses as needed
        return path

    def run(self):
        while not rospy.is_shutdown() and self.current_step < len(self.path):
            desired_pose = self.path[self.current_step]
            self.pose_pub.publish(desired_pose)
            rospy.loginfo(f"Published Pose {self.current_step}: {desired_pose}")
            self.current_step += 1
            self.rate.sleep()
        
        rospy.loginfo("Completed massage path.")

if __name__ == "__main__":
    planner = PathPlanner()
    planner.run()
