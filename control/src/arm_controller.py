import sys
import time
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_srvs.srv import Empty
import actionlib
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from geometry_msgs.msg import PoseStamped


class ArmController(object):
  """ArmController"""
  def __init__(self):

    # Initialize the node
    super(ArmController, self).__init__()
    rospy.init_node('arm_controller')

    moveit_commander.roscpp_initialize(sys.argv)

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

      # Action client for the impedance controller
      self.client = actionlib.SimpleActionClient(
          '/joint_impedance_controller/follow_joint_trajectory',
          FollowJointTrajectoryAction
      )
      rospy.loginfo("Waiting for impedance controller action server...")
      self.client.wait_for_server()
      rospy.loginfo("Connected to impedance controller action server.")

      # Subscribe to desired joint positions
      rospy.Subscriber('/pose_estimation', PoseStamped, self.pose_callback)

      rospy.loginfo("Initializing node in namespace " + rospy.get_namespace())
    except Exception as e:
      print (e)
      self.is_init_success = False
    else:
      self.is_init_success = True

  def pose_callback(self, pose_msg):
    # Get the desired end-effector pose from pose_msg
    desired_pose = pose_msg.pose

    # Compute inverse kinematics to find the joint positions to reach desired_pose
    joint_positions = self.compute_ik(desired_pose)

    if joint_positions is not None:
        # Send the joint positions to the impedance controller
        self.send_joint_trajectory(joint_positions, time_from_start=5.0)
    else:
        rospy.logwarn("Failed to compute IK for desired pose")

  def compute_ik(self, desired_pose):
    # Set the desired pose as the target
    self.arm_group.set_pose_target(desired_pose)
    plan = self.arm_group.plan()
    if plan and len(plan.joint_trajectory.points) > 0:
        # Get the joint positions from the first point of the plan
        joint_positions = plan.joint_trajectory.points[-1].positions
        return joint_positions
    else:
        return None
    
  def send_joint_trajectory(self, joint_positions, time_from_start):
      goal = FollowJointTrajectoryGoal()
      goal.trajectory = JointTrajectory()
      goal.trajectory.joint_names = [
          'joint_1', 'joint_2', 'joint_3', 'joint_4',
          'joint_5', 'joint_6', 'joint_7'
      ]

      point = JointTrajectoryPoint()
      point.positions = joint_positions
      point.time_from_start = rospy.Duration(time_from_start)
      goal.trajectory.points.append(point)

      self.client.send_goal(goal)
      self.client.wait_for_result()
      result = self.client.get_result()
      rospy.loginfo("Trajectory execution result: %s", result)


  def reach_named_position(self, target):
    arm_group = self.arm_group
    
    # Going to one of those targets
    rospy.loginfo("Going to named target " + target)
    # Set the target
    arm_group.set_named_target(target)
    # Plan the trajectory
    (success_flag, trajectory_message, planning_time, error_code) = arm_group.plan()
    # Execute the trajectory and block while it's not finished
    return arm_group.execute(trajectory_message, wait=True)

  def reach_joint_angles(self, tolerance, joint_positions = []):
    arm_group = self.arm_group
    success = True

    rospy.loginfo("Printing current joint positions before movement :")
    for p in arm_group.get_current_joint_values(): rospy.loginfo(p)

    # Set the goal joint tolerance
    self.arm_group.set_goal_joint_tolerance(tolerance)

    # Set the joint target configuration
    print(joint_positions)
    if joint_positions == []:
      if self.degrees_of_freedom == 7:
        joint_positions[0] = pi/2
        joint_positions[1] = 0
        joint_positions[2] = pi/4
        joint_positions[3] = -pi/4
        joint_positions[4] = 0
        joint_positions[5] = pi/2
        joint_positions[6] = 0.2
      elif self.degrees_of_freedom == 6:
        joint_positions[0] = 0
        joint_positions[1] = 0
        joint_positions[2] = pi/2
        joint_positions[3] = pi/4
        joint_positions[4] = 0
        joint_positions[5] = pi/2

    print(joint_positions)
    arm_group.set_joint_value_target(joint_positions)
    
    # Plan and execute in one command
    success &= arm_group.go(wait=True)

    # Show joint positions after movement
    new_joint_positions = arm_group.get_current_joint_values()
    rospy.loginfo("Printing current joint positions after movement :")
    for p in new_joint_positions: rospy.loginfo(p)
    return success

  def get_cartesian_pose(self):
    arm_group = self.arm_group

    # Get the current pose and display it
    pose = arm_group.get_current_pose()
    rospy.loginfo("Actual cartesian pose is : ")
    rospy.loginfo(pose.pose)

    return pose.pose

  def reach_cartesian_pose(self, pose, tolerance, constraints):
    arm_group = self.arm_group
    
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

  def reach_gripper_position(self, relative_position):
    gripper_group = self.gripper_group
    
    # We only have to move this joint because all others are mimic!
    gripper_joint = self.robot.get_joint(self.gripper_joint_name)
    gripper_max_absolute_pos = gripper_joint.max_bound()
    gripper_min_absolute_pos = gripper_joint.min_bound()
    try:
      val = gripper_joint.move(relative_position * (gripper_max_absolute_pos - gripper_min_absolute_pos) + gripper_min_absolute_pos, True)
      return val
    except:
      return False 

def main():
    controller = ArmController()
    rospy.spin()

if __name__ == '__main__':
    main()
