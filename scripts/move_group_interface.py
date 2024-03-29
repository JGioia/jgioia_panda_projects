#!/usr/bin/env/ python
"""Module that contains the MoveGroupInterface"""

import numpy as np
import sys
import copy
import moveit_commander
import geometry_msgs.msg
from moveit_commander.conversions import pose_to_list
import traceback
from franka_gripper.msg import MoveGoal, MoveAction, StopAction, StopGoal
import actionlib
import time


def all_close(goal, actual, tolerance):
  """Convenience method for testing if a list of values are within a tolerance of their counterparts in another list
  
  Args:
    goal (list[float] or Pose or PoseStamped): The goal pose
    actual (list[float] or Pose or PoseStamped): The actual pose
    tolerance (float): The tolerance distances
    
  Returns:
    A bool. This represents whether the goal is within tolerances of the actual
  """
  if type(goal) is list:
    for index in range(len(goal)):
      if abs(actual[index] - goal[index]) > tolerance:
        return False

  elif type(goal) is geometry_msgs.msg.PoseStamped:
    return all_close(goal.pose, actual.pose, tolerance)

  elif type(goal) is geometry_msgs.msg.Pose:
    return all_close(pose_to_list(goal), pose_to_list(actual), tolerance)

  return True


def rpy_to_quaternion(roll, pitch, yaw):
  """Converts rpy angle form to quaternion angle form.
  
  This function was paraphrased from Wikipedia. 
  https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles

  Args:
    roll (float): The angle with respect to the x axis
    pitch (float): The angle with respect to the y axis
    yaw (float): The angle with respect to the z axis

  Returns:
    A quaternion with the same orientation as the input.
  """
  cy = np.cos(yaw * 0.5)
  sy = np.sin(yaw * 0.5)
  cp = np.cos(pitch * 0.5)
  sp = np.sin(pitch * 0.5)
  cr = np.cos(roll * 0.5)
  sr = np.sin(roll * 0.5)

  orientation = geometry_msgs.msg.Quaternion()
  orientation.w = cr * cp * cy + sr * sp * sy
  orientation.x = sr * cp * cy - cr * sp * sy
  orientation.y = cr * sp * cy + sr * cp * sy
  orientation.z = cr * cp * sy - sr * sp * cy

  return orientation

def confirm_message(message):
  """Asks the user for confirmation about an action

  Args:
    message (str): Warning message that describes the action that is about to take place

  Returns: True if the user confirms the action. False if the user denies. 
    Does not return until user inputs a value
  """
  print(message)
  try:
    result = raw_input("Do you want to proceed? (y/n)")
    if (result == "y" or result == "Y"):
      return True
  except:
    return False
  return False


class MoveGroupInterface():
  """Provides useful methods for robot interaction that utilize MoveIt's MoveGroupCommander"""

  def __init__(self):
    """Initializes the interface to control a Franka Emika Panda robot"""

    # First initialize `moveit_commander`:
    moveit_commander.roscpp_initialize(sys.argv)

    # Instantiate a `RobotCommander`_ object. Provides information such as the robot's
    # kinematic model and the robot's current joint states
    self.robot = moveit_commander.RobotCommander()

    # Instantiate a `MoveGroupCommander`_ object.  This object is an interface
    # to a planning group (group of joints).
    # This interface can be used to plan and execute motions:
    self.move_group_arm = moveit_commander.MoveGroupCommander("panda_arm")
    self.move_group_hand = moveit_commander.MoveGroupCommander("hand")

    # We can get the name of the reference frame for this robot:
    self.planning_frame = self.move_group_arm.get_planning_frame()
    # print("Planning frame: " + self.planning_frame)

    # The end-effector link for this group:
    self.eef_link = self.move_group_arm.get_end_effector_link()
    # print("End effector link: " + self.eef_link)

    # We can get a list of all the groups in the robot:
    self.group_names = self.robot.get_group_names()
    # print("Available Planning Groups:", self.robot.get_group_names())

    # Set the max velocity factor to 1 by default
    self.move_group_arm.set_max_velocity_scaling_factor(0.1)
    self.move_group_arm.set_max_acceleration_scaling_factor(0.4)

    # Set the clients for the franka gripper actions
    self.move_client = actionlib.SimpleActionClient('/franka_gripper/move', MoveAction)
    self.move_client.wait_for_server()
    self.stop_client = actionlib.SimpleActionClient('/franka_gripper/stop', StopAction)
    self.stop_client.wait_for_server()

  def go_to_pose(self,
                 x=None,
                 y=None,
                 z=None,
                 roll=None,
                 pitch=None,
                 yaw=None,
                 pose=None):
    """Makes end effector go to the pose position specified by the parameters
    
    Args:
      x (float or None): The goal x position. If None, then goal position is current position.
      y (float or None): The goal y position. If None, then goal position is current position.
      z (float or None): The goal z position. If None, then goal position is current position.
      roll (float or None): The goal roll in radians. If None, then goal roll = 0
      pitch (float or None): The goal pitch in radians. If None, then goal pitch = pi
      yaw (float or None): The goal yaw in radians. If None, then goal yaw = pi / 4
      pose (Pose): Overrides all other parameters if set.
      
    Returns:
      A bool that represents whether the expected state was reached before the timeout.
    """
    # Set the pose goal
    pose_goal = geometry_msgs.msg.Pose()

    # Deal with the case where an orientation is not set
    if roll == None:
      roll = 0
    if pitch == None:
      pitch = np.pi
    if yaw == None:
      yaw = 0.25 * np.pi
    pose_goal.orientation = rpy_to_quaternion(roll, pitch, yaw)

    # Deal with the case where a position is not set
    current_pose = self.move_group_arm.get_current_pose().pose
    if x == None:
      pose_goal.position.x = current_pose.position.x
    else:
      pose_goal.position.x = x
    if y == None:
      pose_goal.position.y = current_pose.position.y
    else:
      pose_goal.position.y = y
    if z == None:
      pose_goal.position.z = current_pose.position.z
    else:
      pose_goal.position.z = z

    # If pose is set, override all other parameters
    if (pose != None):
      pose_goal = pose

    # Now, we call the planner to compute the plan and execute it.
    if (not confirm_message("You are about to move the robot to the pose:\n" + str(pose_goal))):
      return False
    self.move_group_arm.set_pose_target(pose_goal)
    self.move_group_arm.go(wait=True)

    # Calling `stop()` ensures that there is no residual movement
    self.move_group_arm.stop()

    # It is always good to clear your targets after planning with poses.
    # Note: there is no equivalent function for clear_joint_value_targets()
    self.move_group_arm.clear_pose_targets()

    # We use the class variable rather than the copied state variable
    current_pose = self.move_group_arm.get_current_pose().pose

    # Returns if the position we went to is within tolerances of the position we wanted
    return all_close(pose_goal,
                     self.move_group_arm.get_current_pose().pose, 0.01)

  def stop(self):
    """Stops the robot and cancels ongoing actions"""
    self.move_group_arm.stop()
    self.move_group_hand.stop()

  def go_to_joint_goal(self, joint_goal):
    """Moves the joints to the positions specified

    Args:
      joint_goal (list[float or None]): List of 2 joint positions to go to. If a joint position is
        None then the goal for that joint is the current position.

    Returns:
      A bool that represents whether the expected state was reached before the timeout.
    """
    # Deals with the case where a joint's goal is None
    current_joints = self.move_group_arm.get_current_joint_values()
    joint_goal = copy.deepcopy(joint_goal)
    for i in range(len(joint_goal)):
      if joint_goal[i] == None:
        joint_goal[i] = current_joints[i]

    # Moves to joint_goal
    self.move_group_arm.go(joint_goal, wait=True)

    # Ensures that there is no residual movement
    self.move_group_arm.stop()

    # Returns whether the position we went to is within tolerances of the position we wanted
    return all_close(joint_goal, self.move_group_arm.get_current_joint_values(),
                     0.01)

  def go_to_hand_joint_goal(self, joint_goal):
    """Moves the joints in the hand move group to the positions specified

    Args:
      joint_goal (list[float or None]): List of 7 joint positions to go to. If a joint position is
        None then the goal for that joint is the current position.

    Returns:
      A bool that represents whether the expected state was reached before the timeout.
    """
    # Deals with the case where a joint's goal is None
    current_joints = self.move_group_hand.get_current_joint_values()
    joint_goal = copy.deepcopy(joint_goal)
    for i in range(len(joint_goal)):
      if joint_goal[i] == None:
        joint_goal[i] = current_joints[i]

    # Moves to joint_goal
    self.move_group_hand.go(joint_goal, wait=True)

    # Ensures that there is no residual movement
    self.move_group_hand.stop()

    # Returns whether the position we went to is within tolerances of the position we wanted
    return all_close(joint_goal,
                     self.move_group_hand.get_current_joint_values(), 0.01)

  def grab_object(self, item):
    """Grabs the MoveItObject provided
    
    Uses the MoveItObject's grasp_offset and grasp_width to determine
    how to grasp. Does not move if grasp_offset is None and does not
    grip if grasp_width is None. Does not support non default orientations
    currently. Pre-grasp pose is 0.1 above grasp pose with gripper open.

    Args:
      item (MoveItObject): The object to grab.
    """
    self.open_gripper()
    if (item.grasp_offset != None):
      x_pos = item.grasp_offset.position.x + item.pose.pose.position.x
      y_pos = item.grasp_offset.position.y + item.pose.pose.position.y
      z_pos = item.grasp_offset.position.z + item.pose.pose.position.z

      print("Moving to pregrasp position 0.1 above grasp position")
      if (not self.go_to_pose(x=x_pos, y=y_pos, z=z_pos + 0.1)):
        print("Pregrasp move failed")
        # TODO: This is returning false more than i would expect
        # Once resolve add return False here
      print("Moving to grasp position")
      if (not self.go_to_pose(x=x_pos, y=y_pos, z=z_pos)):
        print("Grasp move failed")
    if (not confirm_message("You are about to start gripping at the current position")):
      return False
    self.start_gripping()
    time.sleep(1)

  def slide_to(self, x, y, rotate=True):
    """Moves horizontally to the x,y position at the current z position. 
    
    Moves in a way such that if it is grasping a box, the box will move
    with it.

    Args:
        x (float): The x position to go to
        y (float): The y position to go to
    """
    current_pose = self.move_group_arm.get_current_pose().pose
    x_change = x - current_pose.position.x
    y_change = y - current_pose.position.y
    y_angle = np.arctan(x_change / y_change)

    # Set angle correctly
    if (rotate):
      self.move_group_arm.set_max_velocity_scaling_factor(0.03)
      goal = geometry_msgs.msg.Pose()
      goal.position = current_pose.position
      goal.orientation = rpy_to_quaternion(0, np.pi, y_angle + np.pi / 4)
      path, fraction = self.move_group_arm.compute_cartesian_path([goal], 0.001,
                                                                  0.0)
      self.move_group_arm.execute(path)
      # self.rotate_gripper(y_angle - np.pi / 2)
      self.move_group_arm.set_max_velocity_scaling_factor(1)

    # Follow straight line to goal
    goal = self.move_group_arm.get_current_pose().pose
    goal.position.x = x
    goal.position.y = y
    # goal.position.z = current_pose.position.z
    # goal.orientation = rpy_to_quaternion(0, np.pi, y_angle - np.pi / 4)
    path, fraction = self.move_group_arm.compute_cartesian_path([goal], 0.001,
                                                                0.0)
    self.move_group_arm.execute(path)

  def move_delta(self, move_x, move_y, move_z, delta=0.1):
    """Moves in a straight line to point delta away in each of the specified axis

    Args:
      x (int): 1 if move in +x, -1 if move in -x, 0 if don't move in x-axis
      y (int): 1 if move in +y, -1 if move in -y, 0 if don't move in y-axis
      z (int): 1 if move in +z, -1 if move in -z, 0 if don't move in z-axis
      delta (float): amount to move by in each axis (must be <= 0.2)

    Returns boolean of whether operation completed sucessfully
    """
    goal = self.move_group_arm.get_current_pose().pose
    
    # Check validity of delta
    if (delta < 0 or delta > 0.2):
      print("delta must be between 0 and 0.2")
      print("delta recieved: ", str(delta))
      traceback.print_stack()
      return False

    # Check validity of axis and set goal
    if (move_x == 1):
      goal.position.x += delta
    elif (move_x == -1):
      goal.position.x -= delta
    elif (move_x != 0):
      print("move_x must be 0, 1, or -1")
      print("move_x recieved: ", str(move_x))
      traceback.print_stack()
      return False

    if (move_y == 1):
      goal.position.y += delta
    elif (move_y == -1):
      goal.position.y -= delta
    elif (move_y != 0):
      print("move_y must be 0, 1, or -1")
      print("move_y recieved: ", str(move_y))
      traceback.print_stack()
      return False

    if (move_z == 1):
      goal.position.z += delta
    elif (move_z == -1):
      goal.position.z -= delta
    elif (move_z != 0):
      print("move_z must be 0, 1, or -1")
      print("move_z recieved: ", str(move_z))
      traceback.print_stack()
      return False

    # Move to goal
    path, fraction = self.move_group_arm.compute_cartesian_path([goal], 0.001,
                                                                0.0)
    if (not confirm_message("You are about to move the robot to the pose:\n" + str(goal))):
      return False
    self.move_group_arm.execute(path)

    # Return whether at goal
    return all_close(goal, self.move_group_arm.get_current_pose().pose, 0.01)

  def rotate_gripper(self, angle):
    """Rotates the gripper to the specified angle from the x axis.
    
    NOTE: This is currently relative the the grippers original pose,
    so you may get unexpected results when you have changed the 
    orientation of the robot.

    Args:
        angle (float): The angle from the x-axis.
    """
    angle = (angle % np.pi) - np.pi
    zero_joint = 2
    neg_pi_joint = -1.1

    joint_goal = zero_joint + (angle / np.pi) * (zero_joint - neg_pi_joint)
    self.go_to_joint_goal([None, None, None, None, None, None, joint_goal])

  def open_gripper(self):
    """Opens the gripper. Returns after the action is complete.

    Returns:
      A bool that represents whether the gripper was fully opened
    """
    return self.go_to_hand_joint_goal([0.035, 0.035])

  def close_gripper(self):
    """Closes the gripper. Returns after the action is complete.

    Returns:
      A bool that represents whether the gripper was fully closed
    """
    return self.go_to_hand_joint_goal([0, 0])

  def start_gripping(self):
    """Starts closing the gripper and does not stop when the function returns.

    Stops when stop_gripping is called, another gripper action (like open or close) is initiated
    or the gripper is completely closed.
    """
    goal = MoveGoal(width = 0, speed = 0.08)
    self.move_client.send_goal(goal)

  def stop_gripping(self):
    """Stops closing the gripper.
    """
    goal = StopGoal()
    self.stop_client.send_goal(goal)

  def test(self):
    """A method used for feature testing. Currently not testing anything."""
    pass
