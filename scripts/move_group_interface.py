"""Module that contains the MoveGroupInterface"""

import numpy as np
import sys
import copy
import rospy
import time
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from moveit_commander.conversions import pose_to_list


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


class MoveGroupInterface():
  """Provides useful methods for robot interaction that utilize the MoveGroupCommander class"""

  def __init__(self):
    """Initializes the interface to control a Franka Emika Panda robot"""

    # First initialize `moveit_commander`_ and a `rospy`_ node:
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('move_group_interface', anonymous=True)

    # Instantiate a `RobotCommander`_ object. Provides information such as the robot's
    # kinematic model and the robot's current joint states
    self.robot = moveit_commander.RobotCommander()

    # Instantiate a `PlanningSceneInterface`_ object.  This provides a remote interface
    # for getting, setting, and updating the robot's internal understanding of the
    # surrounding world:
    self.scene = moveit_commander.PlanningSceneInterface()

    # Instantiate a `MoveGroupCommander`_ object.  This object is an interface
    # to a planning group (group of joints).
    # This interface can be used to plan and execute motions:
    self.move_group_arm = moveit_commander.MoveGroupCommander("panda_arm")
    self.move_group_hand = moveit_commander.MoveGroupCommander("hand")

    # Create a `DisplayTrajectory`_ ROS publisher which is used to display
    # trajectories in Rviz:
    self.display_trajectory_publisher = rospy.Publisher(
        '/move_group/display_planned_path',
        moveit_msgs.msg.DisplayTrajectory,
        queue_size=20)

    # We can get the name of the reference frame for this robot:
    self.planning_frame = self.move_group_arm.get_planning_frame()
    # print("Planning frame: " + self.planning_frame)

    # We can also print the name of the end-effector link for this group:
    self.eef_link = self.move_group_arm.get_end_effector_link()
    # print("End effector link: " + self.eef_link)

    # We can get a list of all the groups in the robot:
    self.group_names = self.robot.get_group_names()
    # print("Available Planning Groups:", self.robot.get_group_names())

    # Sometimes for debugging it is useful to print the entire state of the
    # robot:
    # print("Printing robot state")
    # print(self.robot.get_current_state())
    # print("")

    # Misc variables
    self.box_name = ''

    self.box_pose = geometry_msgs.msg.PoseStamped()
    self.box_pose.header.frame_id = "panda_link0"
    self.box_pose.pose.orientation.w = 1.0

    # self.scene.add_mesh(
    # "table", [0, 0, 0],
    # "/home/joseph/ws_moveit/src/jgioia_panda_projects/models/table.STL",
    # size=(0.001, 0.001, 0.001))

  def go_to_pose(self, x=None, y=None, z=None):
    """Makes end effector go to the pose position specified by the parameters
    
    Args:
      x (float or None): The goal x position. If None, then goal position is current position.
      y (float or None): The goal y position. If None, then goal position is current position.
      z (float or None): The goal z position. If None, then goal position is current position.
      
    Returns:
      A bool that represents whether the expected state was reached before the timeout.
    """
    # Set the pose goal
    pose_goal = geometry_msgs.msg.Pose()

    pose_goal.orientation = rpy_to_quaternion(0, np.pi, 0.25 * np.pi)

    # Deals with the case where a parameter is not set
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

    # Now, we call the planner to compute the plan and execute it.
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

  def go_to_joint_goal(self, joint_goal):
    """Moves the joints to the positions specified

    Args:
      joint_goal (list[float or None]): List of joint positions to go to. If a joint position is
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
      joint_goal (list[float or None]): List of joint positions to go to. If a joint position is
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

  def wait_for_state_update(self,
                            box_is_known=False,
                            box_is_attached=False,
                            timeout=4):
    """Waits until we reach the expected state.

    Args:
      box_is_known (bool): Optional; Expected value for whether the box is in the scene. 
        Defaults to False.
      box_is_attached (bool): Optional; Expected value for whether the box is attached 
        to the robot. Defaults to False.
      timeout (int): Optional; Number of seconds to wait as a maximum for the expected
        state to be reached. Defaults to 4.

    Returns:
      A bool that represents whether the expected state was reached before the timeout.
    """
    start = rospy.get_time()
    seconds = rospy.get_time()

    while (seconds - start < timeout) and not rospy.is_shutdown():
      # Test if the box is in attached objects
      attached_objects = self.scene.get_attached_objects([self.box_name])
      is_attached = len(attached_objects.keys()) > 0

      # Test if the box is in the scene.
      # Note that attaching the box will remove it from known_objects
      is_known = self.box_name in self.scene.get_known_object_names()

      # Test if we are in the expected state
      if (box_is_attached == is_attached) and (box_is_known == is_known):
        return True

      # Sleep so that we give other threads time on the processor
      rospy.sleep(0.1)
      seconds = rospy.get_time()

    # If we exited the while loop without returning then we timed out
    return False

  def add_box(self, x, y, z, timeout=4):
    """Adds a box at the location of the panda's left box stores it in an object variable.

    Args:
      timeout (int): Optional; Number of seconds to wait as a maximum for the box to
        be added. Defaults to 4.

    Returns:
      A bool that represents whether the box was successfully added
    """
    self.box_pose.pose.position.x = x
    self.box_pose.pose.position.y = y
    self.box_pose.pose.position.z = z
    self.box_name = "box"

    self.scene.add_box(self.box_name, self.box_pose, size=(0.03, 0.03, 0.03))
    return self.wait_for_state_update(box_is_known=True, timeout=timeout)

  def grab_box(self):
    """Grabs the box assuming it has been added.
    
    Only works for a box of size (0.03, 0.03, 0.03) in the default orientation.
    """
    self.open_gripper()
    self.go_to_pose(self.box_pose.pose.position.x,
                    self.box_pose.pose.position.y,
                    self.box_pose.pose.position.z + 0.1)
    self.go_to_hand_joint_goal([0.0, 0.0])
    self.attach_box()

  def attach_box(self, timeout=4):
    """Attaches the box to the Panda wrist.

    Args:
      timeout (int): Optional; Number of seconds to wait as a maximum for the box to
        be attached. Defaults to 4.

    Returns:
      A bool that represents whether the box was successfully attached
    """
    # Attaches the box to the hand group of the robot
    grasping_group = 'hand'
    touch_links = self.robot.get_link_names(group=grasping_group)
    self.scene.attach_box("panda_link8", self.box_name)

    # Checks whether the box was successfully added
    return self.wait_for_state_update(box_is_attached=True,
                                      box_is_known=False,
                                      timeout=timeout)

  def detach_box(self, timeout=4):
    """Detaches the box from the Panda wrist.

    Args:
      timeout (int): Optional; Number of seconds to wait as a maximum for the box to
        be detached. Defaults to 4.

    Returns:
        A bool that represents whether the box was successfully detached
    """
    # Detach the box
    self.scene.remove_attached_object(self.eef_link, name=self.box_name)

    # Checks whether the box was successfully detached
    return self.wait_for_state_update(box_is_known=True,
                                      box_is_attached=False,
                                      timeout=timeout)

  def remove_box(self, timeout=4):
    """Remove the box from the planning scene. The box must be detached to do this.

    Args:
      timeout (int): Optional; Number of seconds to wait as a maximum for the box to
        be removed. Defaults to 4.

    Returns:
        A bool that represents whether the box was successfully removed
    """
    # Make sure box is detached
    self.detach_box()

    # Remvoe the box from the world
    self.scene.remove_world_object(self.box_name)

    # We wait for the planning scene to update.
    return self.wait_for_state_update(box_is_attached=False,
                                      box_is_known=False,
                                      timeout=timeout)

  def open_gripper(self):
    """Opens the gripper

    Returns:
      A bool that represents whether the gripper was fully opened
    """
    return self.go_to_hand_joint_goal([0.035, 0.035])

  def close_gripper(self):
    """Closes the gripper

    Returns:
      A bool that represents whether the gripper was fully closed
    """
    return self.go_to_hand_joint_goal([0, 0])

  def test(self):
    """A method to be used for feature testing. Currently testing gazebo."""
    print("open")
    self.open_gripper()
    print("move")
    self.go_to_pose(0.42, 0, 0.2)
    print("move")
    self.go_to_pose(0.42, 0, 0.13)
    print("close")
    self.close_gripper()
    print("move")
    self.go_to_pose(0.42, 0, 0.3)
