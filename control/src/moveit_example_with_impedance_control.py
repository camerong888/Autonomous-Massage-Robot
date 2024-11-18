#!/usr/bin/env python3

import sys
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_srvs.srv import Empty
from moveit_msgs.msg import RobotTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
import actionlib
from moveit_msgs.msg import DisplayTrajectory
from std_msgs.msg import Header
from moveit_msgs.msg import CartesianTrajectoryPoint

class ArmController(object):
    """ArmController using MoveIt! and impedance controller"""
    def __init__(self):
        # Initialize the node
        super(ArmController, self).__init__()
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('arm_controller')

        # Parameters
        self.degrees_of_freedom = rospy.get_param(rospy.get_namespace() + "degrees_of_freedom", 7)

        # Create MoveIt! interfaces
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        arm_group_name = "arm"
        self.arm_group = moveit_commander.MoveGroupCommander(arm_group_name)
        self.arm_group.set_planning_time(10)
        self.arm_group.set_num_planning_attempts(10)
        self.display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                            moveit_msgs.msg.DisplayTrajectory,
                                                            queue_size=20)

        # Publisher to impedance controller in task space
        self.cmd_pub = rospy.Publisher('/task_space_compliant_controller/command', CartesianTrajectoryPoint, queue_size=1)

        # Initialize variables
        self.is_init_success = True

    def plan_cartesian_path(self, waypoints):
        # Plan a Cartesian path connecting the waypoints
        (plan, fraction) = self.arm_group.compute_cartesian_path(
            waypoints,
            0.01,  # eef_step
            0.0    # jump_threshold
        )
        return plan, fraction

    def execute_plan(self, plan):
        # Extract waypoints and send them to the impedance controller
        if plan and len(plan.multi_dof_joint_trajectory.points) > 0:
            rospy.loginfo("Executing plan via impedance controller")
            for point in plan.multi_dof_joint_trajectory.points:
                cmd = CartesianTrajectoryPoint()
                cmd.point.pose = point.transforms[0]
                cmd.time_from_start = point.time_from_start
                self.cmd_pub.publish(cmd)
                rospy.sleep(0.01)  # Adjust sleep time as necessary
        else:
            rospy.logwarn("Plan is empty or invalid")

def main():
    arm_controller = ArmController()

    # For testing purposes
    success = arm_controller.is_init_success

    if success:
        rospy.loginfo("Planning Cartesian Path...")

        # Define waypoints
        waypoints = []

        # Start with the current pose
        start_pose = arm_controller.arm_group.get_current_pose().pose
        waypoints.append(start_pose)

        # Define the target pose
        target_pose = geometry_msgs.msg.Pose()
        target_pose.orientation = start_pose.orientation
        target_pose.position.x = start_pose.position.x
        target_pose.position.y = start_pose.position.y - 0.1  # Move left
        target_pose.position.z = start_pose.position.z - 0.1  # Move down
        waypoints.append(target_pose)

        # Plan the Cartesian path connecting the waypoints
        plan, fraction = arm_controller.plan_cartesian_path(waypoints)

        rospy.loginfo("Cartesian path planned with fraction: {}".format(fraction))

        if fraction > 0.9:
            # Visualize the plan
            display_trajectory = DisplayTrajectory()
            display_trajectory.trajectory_start = arm_controller.robot.get_current_state()
            display_trajectory.trajectory.append(plan)
            arm_controller.display_trajectory_publisher.publish(display_trajectory)
            rospy.sleep(2)

            # Execute the plan via impedance controller
            arm_controller.execute_plan(plan)
        else:
            rospy.logwarn("Cartesian path planning failed with low fraction")

if __name__ == '__main__':
    main()
