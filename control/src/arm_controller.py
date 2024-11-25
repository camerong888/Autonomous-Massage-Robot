#!/usr/bin/env python3

import rospy
import sys
import tf2_ros
import tf2_geometry_msgs
from geometry_msgs.msg import PoseStamped, Quaternion
import moveit_commander
import moveit_msgs.msg
from math import pi
import threading
import numpy as np
import tf.transformations as tf_trans


'''
  position: 
    x: 0.25694293215954833
    y: 0.16662751870909523
    z: 0.5883970690718603
  orientation: 
    x: 0.5857211500067097
    y: 0.7815929463924565
    z: 0.1894087894033471
    w: 0.10083407698360528

    '''


'''
position: 
  x: 0.6215961507577014
  y: 0.5304335731563292
  z: -0.33493609868494345
orientation: 
  x: 1.0
  y: 0.0
  z: 0.0
  w: 6.123233995736766e-17, position: 
  x: 0.4064084115592175
  y: 0.46020739844571545
  z: -0.35391962583138525
'''
class ArmController:
    def __init__(self):
        # Initialize the node
        rospy.init_node('arm_controller', anonymous=True)

        # Initialize MoveIt!
        moveit_commander.roscpp_initialize(sys.argv)
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        self.group_name = "arm"
        self.move_group = moveit_commander.MoveGroupCommander(self.group_name)
        self.move_group.set_planning_time(10)
        self.move_group.set_num_planning_attempts(10)
        self.move_group.set_max_velocity_scaling_factor(0.1)  # Adjust as needed
        self.move_group.set_max_acceleration_scaling_factor(0.1)  # Adjust as needed

        # Initialize TF buffer and listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        # List of joint frames to listen to
        # self.joint_frames = [f"joint_{i}" for i in range(11)]  # Adjust range as needed
        self.joint_frames = ['joint_0', 'joint_2', 'joint_4']

        # Coordinate frames
        self.robot_base_frame = "base_link"  # Adjust if your robot's base frame is different

        # Run the control loop in a separate thread
        # Get the current pose and display it
        self.initial_pose = self.move_group.get_current_pose()
        print(self.initial_pose)


        self.control_thread = threading.Thread(target=self.control_loop)
        self.control_thread.start()

    def control_loop(self):
        rate = rospy.Rate(0.2)  # Control loop rate in Hz (once every 5 seconds)
        # while not rospy.is_shutdown():
        # Get the latest joint positions (should this happen every loop??)
        joint_positions = self.get_joint_positions()

        if joint_positions:
            rospy.loginfo("Planning path through joint positions")
            # Plan and execute the motion through all joint positions
            # success = self.move_through_joint_positions(joint_positions)
            for pose in joint_positions:
                print(pose)
                pose = joint_positions[pose]
                self.reach_cartesian_pose(pose)

                success = True
            if success:
                rospy.loginfo("Motion executed successfully")
            else:
                rospy.logwarn("Motion planning failed")
        else:
            rospy.logwarn("No joint positions available")

            rate.sleep()


    def reach_cartesian_pose(self, pose, tolerance=0.01, constraints=None):

        print(pose)
        arm_group = self.move_group
        
        # Set the tolerance
        arm_group.set_goal_position_tolerance(tolerance)

        # Set the trajectory constraint if one is specified
        if constraints is not None:
            arm_group.set_path_constraints(constraints)

        # Get the current Cartesian Position
        arm_group.set_pose_target(pose)

        # Plan and execute
        rospy.loginfo("Planning and going to the Cartesian Pose")
        return arm_group.go(wait=True)

    def get_joint_positions(self):
        joint_positions = {}
        for frame in self.joint_frames:
            try:
                # Lookup transform from robot base frame to joint frame
                trans = self.tf_buffer.lookup_transform(
                    self.robot_base_frame,                 # target frame
                    frame,  # source frame
                    rospy.Time(0),         # get the latest available transform
                    rospy.Duration(1.0)    # timeout
                )

                # Convert transform to PoseStamped
                pose = PoseStamped()
                pose.header.frame_id = self.robot_base_frame
                pose.header.stamp = rospy.Time.now()
                pose.pose.position = trans.transform.translation

                # Set end effector orientation to point down
                # Quaternion representing rotation of 180 degrees around X-axis
                # q_down = tf_trans.quaternion_from_euler(0, pi, 0)
                # pose.pose.orientation = Quaternion(*q_down)
                pose.pose.orientation = self.initial_pose.pose.orientation

                joint_positions[frame] = pose

            except (tf2_ros.LookupException, tf2_ros.ExtrapolationException, tf2_ros.ConnectivityException) as e:
                rospy.logwarn(f"TF lookup failed for {frame}: {e}")
                continue

        return joint_positions

    def move_through_joint_positions(self, joint_positions):
        move_group = self.move_group

        # Sort the joint positions by joint index to ensure consistent order
        sorted_joints = sorted(joint_positions.items(), key=lambda x: int(x[0].split('_')[1]))

        # Create waypoints
        waypoints = []
        for frame, pose in sorted_joints:
            waypoints.append(pose.pose)

        if not waypoints:
            rospy.logwarn("No valid waypoints available")
            return False

        
        print(waypoints)

        # Plan Cartesian path through waypoints
        (plan, fraction) = move_group.compute_cartesian_path(
            waypoints,   # waypoints to follow
            0.02,        # eef_step in meters
            avoid_collisions=True
        )

        print(plan)

        rospy.loginfo(f"Cartesian path planning fraction: {fraction}")

        # if fraction < 0.9:
        #     rospy.logwarn("Unable to plan complete Cartesian path")
        #     return False

        # Execute the plan
        move_group.execute(plan, wait=True)

        # Clear targets
        move_group.stop()
        move_group.clear_pose_targets()

        return True

if __name__ == "__main__":
    try:
        arm_controller = ArmController()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
