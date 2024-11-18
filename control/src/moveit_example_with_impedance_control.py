#!/usr/bin/env python3

import sys
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_srvs.srv import Empty
import actionlib
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal

class ExampleMoveItTrajectories(object):
    """ExampleMoveItTrajectories with Impedance Controller"""
    def __init__(self):
        # Initialize the node
        super(ExampleMoveItTrajectories, self).__init__()
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('example_move_it_trajectories')

        try:
            self.is_gripper_present = rospy.get_param(rospy.get_namespace() + "is_gripper_present", False)
            if self.is_gripper_present:
                gripper_joint_names = rospy.get_param(rospy.get_namespace() + "gripper_joint_names", [])
                self.gripper_joint_name = gripper_joint_names[0]
            else:
                self.gripper_joint_name = ""
            self.degrees_of_freedom = rospy.get_param(rospy.get_namespace() + "degrees_of_freedom", 7)

            # Create the MoveItInterface necessary objects
            arm_group_name = "arm"
            self.robot = moveit_commander.RobotCommander("robot_description")
            self.scene = moveit_commander.PlanningSceneInterface(ns=rospy.get_namespace())
            self.arm_group = moveit_commander.MoveGroupCommander(arm_group_name, ns=rospy.get_namespace())
            self.display_trajectory_publisher = rospy.Publisher(rospy.get_namespace() + 'move_group/display_planned_path',
                                                           moveit_msgs.msg.DisplayTrajectory,
                                                           queue_size=20)

            if self.is_gripper_present:
                gripper_group_name = "gripper"
                self.gripper_group = moveit_commander.MoveGroupCommander(gripper_group_name, ns=rospy.get_namespace())

            # Initialize impedance controller action client
            self.impedance_client = actionlib.SimpleActionClient(
                '/joint_impedance_controller/follow_joint_trajectory',
                FollowJointTrajectoryAction
            )
            rospy.loginfo("Waiting for impedance controller action server...")
            self.impedance_client.wait_for_server()
            rospy.loginfo("Connected to impedance controller action server.")

            rospy.loginfo("Initializing node in namespace " + rospy.get_namespace())
        except Exception as e:
            print(e)
            self.is_init_success = False
        else:
            self.is_init_success = True

    def send_trajectory_to_impedance_controller(self, plan):
        # Create a FollowJointTrajectoryGoal
        goal = FollowJointTrajectoryGoal()
        goal.trajectory = plan.joint_trajectory

        # Send the goal to the impedance controller
        self.impedance_client.send_goal(goal)
        self.impedance_client.wait_for_result()
        result = self.impedance_client.get_result()
        rospy.loginfo("Trajectory execution result: %s", result)

    def reach_named_position(self, target):
        arm_group = self.arm_group

        # Going to one of those targets
        rospy.loginfo("Going to named target " + target)
        # Set the target
        arm_group.set_named_target(target)
        # Plan the trajectory
        plan = arm_group.plan()
        # Send the trajectory to the impedance controller
        self.send_trajectory_to_impedance_controller(plan)

    def reach_joint_angles(self, tolerance, joint_positions=[]):
        arm_group = self.arm_group

        rospy.loginfo("Printing current joint positions before movement:")
        for p in arm_group.get_current_joint_values():
            rospy.loginfo(p)

        # Set the goal joint tolerance
        arm_group.set_goal_joint_tolerance(tolerance)

        # Set the joint target configuration
        if not joint_positions:
            if self.degrees_of_freedom == 7:
                joint_positions = [pi/2, 0, pi/4, -pi/4, 0, pi/2, 0.2]
            elif self.degrees_of_freedom == 6:
                joint_positions = [0, 0, pi/2, pi/4, 0, pi/2]

        arm_group.set_joint_value_target(joint_positions)

        # Plan the trajectory
        plan = arm_group.plan()
        # Send the trajectory to the impedance controller
        self.send_trajectory_to_impedance_controller(plan)

        # Show joint positions after movement
        new_joint_positions = arm_group.get_current_joint_values()
        rospy.loginfo("Printing current joint positions after movement:")
        for p in new_joint_positions:
            rospy.loginfo(p)

    def get_cartesian_pose(self):
        arm_group = self.arm_group

        # Get the current pose and display it
        pose = arm_group.get_current_pose()
        rospy.loginfo("Actual cartesian pose is:")
        rospy.loginfo(pose.pose)

        return pose.pose

    def reach_cartesian_pose(self, pose, tolerance, constraints):
        arm_group = self.arm_group

        # Set the tolerance
        arm_group.set_goal_position_tolerance(tolerance)

        # Set the trajectory constraint if one is specified
        if constraints is not None:
            arm_group.set_path_constraints(constraints)

        # Set the pose target
        arm_group.set_pose_target(pose)

        # Plan the trajectory
        plan = arm_group.plan()
        # Send the trajectory to the impedance controller
        self.send_trajectory_to_impedance_controller(plan)

    def reach_gripper_position(self, relative_position):
        if not self.is_gripper_present:
            rospy.logwarn("Gripper is not present on the robot.")
            return False

        gripper_group = self.gripper_group

        # We only have to move this joint because all others are mimic!
        gripper_joint = self.robot.get_joint(self.gripper_joint_name)
        gripper_max_absolute_pos = gripper_joint.max_bound()
        gripper_min_absolute_pos = gripper_joint.min_bound()
        try:
            val = gripper_joint.move(relative_position * (gripper_max_absolute_pos - gripper_min_absolute_pos) + gripper_min_absolute_pos, True)
            return val
        except Exception as e:
            rospy.logerr("Failed to move gripper: %s", e)
            return False

def main():
    example = ExampleMoveItTrajectories()

    # For testing purposes
    success = example.is_init_success
    try:
        rospy.delete_param("/kortex_examples_test_results/moveit_general_python")
    except:
        pass

    if success:
        rospy.loginfo("Reaching Joint Angles...")
        look_at_floor_joints = [0, 0, pi, -pi/2, 0, -pi/2, pi/2]
        example.reach_joint_angles(tolerance=0.01, joint_positions=look_at_floor_joints)

    if success:
        rospy.loginfo("Reaching Cartesian Pose...")

        actual_pose = example.get_cartesian_pose()
        actual_pose.position.z -= 0.2
        example.reach_cartesian_pose(pose=actual_pose, tolerance=0.01, constraints=None)

    if example.degrees_of_freedom == 7 and success:
        rospy.loginfo("Reach Cartesian Pose with constraints...")
        # Get actual pose
        actual_pose = example.get_cartesian_pose()
        actual_pose.position.y -= 0.3

        # Orientation constraint (we want the end effector to stay the same orientation)
        constraints = moveit_msgs.msg.Constraints()
        orientation_constraint = moveit_msgs.msg.OrientationConstraint()
        orientation_constraint.orientation = actual_pose.orientation
        constraints.orientation_constraints.append(orientation_constraint)

        # Send the goal
        example.reach_cartesian_pose(pose=actual_pose, tolerance=0.01, constraints=constraints)

    if example.is_gripper_present and success:
        rospy.loginfo("Opening the gripper...")
        example.reach_gripper_position(0)

        rospy.loginfo("Closing the gripper 50%...")
        example.reach_gripper_position(0.5)

    # For testing purposes
    rospy.set_param("/kortex_examples_test_results/moveit_general_python", success)

    if not success:
        rospy.logerr("The example encountered an error.")

if __name__ == '__main__':
    main()
