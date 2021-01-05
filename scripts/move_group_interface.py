#!/usr/bin/env python

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list

def all_close(goal, actual, tolerance):
  """
  Convenience method for testing if a list of values are within a tolerance of their counterparts in another list
  @param: goal       A list of floats, a Pose or a PoseStamped
  @param: actual     A list of floats, a Pose or a PoseStamped
  @param: tolerance  A float
  @returns: bool
  """
  all_equal = True
  if type(goal) is list:
    for index in range(len(goal)):
      if abs(actual[index] - goal[index]) > tolerance:
        return False

  elif type(goal) is geometry_msgs.msg.PoseStamped:
    return all_close(goal.pose, actual.pose, tolerance)

  elif type(goal) is geometry_msgs.msg.Pose:
    return all_close(pose_to_list(goal), pose_to_list(actual), tolerance)

  return True


class MoveGroupInterface():
  """
  Provides useful methods for robot interaction that utilize the MoveGroupCommander class
  """

  def __init__(self):
    """
    Initializes the interface to control a Franka Emika Panda robot
    """
    
    # First initialize `moveit_commander`_ and a `rospy`_ node:
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('move_group_python_interface_tutorial', anonymous=True)

    # Instantiate a `RobotCommander`_ object. Provides information such as the robot's
    # kinematic model and the robot's current joint states
    robot = moveit_commander.RobotCommander()

    # Instantiate a `PlanningSceneInterface`_ object.  This provides a remote interface
    # for getting, setting, and updating the robot's internal understanding of the
    # surrounding world:
    scene = moveit_commander.PlanningSceneInterface()

    # Instantiate a `MoveGroupCommander`_ object.  This object is an interface
    # to a planning group (group of joints). 
    # This interface can be used to plan and execute motions:
    group_name = "panda_arm"
    move_group = moveit_commander.MoveGroupCommander(group_name)

    # Create a `DisplayTrajectory`_ ROS publisher which is used to display
    # trajectories in Rviz:
    display_trajectory_publisher = rospy.Publisher(
      '/move_group/display_planned_path',
      moveit_msgs.msg.DisplayTrajectory,
      queue_size=20)

    # We can get the name of the reference frame for this robot:
    planning_frame = move_group.get_planning_frame()
    print("Planning frame: " + planning_frame)

    # We can also print the name of the end-effector link for this group:
    eef_link = move_group.get_end_effector_link()
    print("End effector link: " + eef_link)

    # We can get a list of all the groups in the robot:
    group_names = robot.get_group_names()
    print("Available Planning Groups:", robot.get_group_names())

    # Sometimes for debugging it is useful to print the entire state of the
    # robot:
    print("Printing robot state")
    print(robot.get_current_state())
    print("")

    # Misc variables
    self.box_name = ''
    self.robot = robot
    self.scene = scene
    self.move_group = move_group
    self.display_trajectory_publisher = display_trajectory_publisher
    self.planning_frame = planning_frame
    self.eef_link = eef_link
    self.group_names = group_names
    

  def go_to_pose(self, x, y, z):
    """
    Makes end effector go to the pose position specified by the parameters
    @param: x   The goal x position
    @param: y   The goal y position
    @param: z   The goal z position
    """
    
    # Set the pose goal
    pose_goal = geometry_msgs.msg.Pose()
    pose_goal.orientation.x = 1
    pose_goal.orientation.y = 1
    pose_goal.orientation.z = 1
    pose_goal.position.x = x
    pose_goal.position.y = y
    pose_goal.position.z = z
    self.move_group.set_pose_target(pose_goal)

    # Now, we call the planner to compute the plan and execute it.
    plan = self.move_group.go(wait=True)
    
    # Calling `stop()` ensures that there is no residual movement
    self.move_group.stop()
    
    # It is always good to clear your targets after planning with poses.
    # Note: there is no equivalent function for clear_joint_value_targets()
    self.move_group.clear_pose_targets()

    # We use the class variable rather than the copied state variable
    current_pose = self.move_group.get_current_pose().pose
    
    # Return if the position we went to is within tolerances of the position we wanted
    return all_close(pose_goal, self.move_group.get_current_pose().pose, 0.01)
  

  def wait_for_state_update(self, box_is_known=False, box_is_attached=False, timeout=4):
    """
    Waits until we reach the expected state. Needed because the node may die before
    publishing a collision object update message.

    Args:
      box_is_known (bool, optional): Expected value for whether the box is in the scene. 
        Defaults to False.
      box_is_attached (bool, optional): Expected value for whether the box is attached 
        to the robot. Defaults to False.
      timeout (int, optional): Number of seconds to wait as a maximum for the expected
        state to be reached. Defaults to 4.

    Returns:
      bool: Whether the expected state was reached before the timeout
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
  

  def add_box(self, timeout=4):
    """
    Adds a box at the location of the panda's left box stores it in an object variable.

    Args:
      timeout (int, optional): Number of seconds to wait as a maximum for the box to
      be added. Defaults to 4.

    Returns:
      bool: Whether the box was successfully added
    """
    
    # Creates the pose for the box to be placed at
    box_pose = geometry_msgs.msg.PoseStamped()
    box_pose.header.frame_id = "panda_link0"
    box_pose.pose.orientation.w = 0
    box_pose.pose.position.x = 0.5
    box_pose.pose.position.y = 0
    box_pose.pose.position.z = 0
    
    # Adds the box
    self.box_name = "box"
    self.scene.add_box(self.box_name, box_pose, size=(0.1, 0.1, 0.1))

    # Checks whether the box was successfully added
    return self.wait_for_state_update(box_is_known=True, timeout=timeout)


  def attach_box(self, timeout=4):
    """
    Attaches the box to the Panda wrist. Tells the planning scene to ignore collisions 
    between the Panda and the box.

    Args:
      timeout (int, optional): Number of seconds to wait as a maximum for the box to
      be attached. Defaults to 4.

    Returns:
      bool: Whether the box was successfully attached
    """

    # Attaches the box to the hand group of the robot
    grasping_group = 'hand'
    touch_links = self.robot.get_link_names(group=grasping_group)
    self.scene.attach_box(self.eef_link, self.box_name, touch_links=touch_links)

    # Checks whether the box was successfully added
    return self.wait_for_state_update(box_is_attached=True, box_is_known=False, timeout=timeout)


  def detach_box(self, timeout=4):
    """
    Detaches the box from the Panda wrist. Tells the planning scene not to ignore collisions
    between the Panda and the box.

    Args:
        timeout (int, optional): Number of seconds to wait as a maximum for the box to
        be detached. Defaults to 4.

    Returns:
        bool: Whether the box was successfully detached
    """

    # Detach the box
    self.scene.remove_attached_object(self.eef_link, name=self.box_name)

    # Checks whether the box was successfully detached
    return self.wait_for_state_update(box_is_known=True, box_is_attached=False, timeout=timeout)


  def remove_box(self, timeout=4):
    """
    Remove the box from the planning scene. The box must be detached to do this.

    Args:
        timeout (int, optional): Number of seconds to wait as a maximum for the box to
        be removed. Defaults to 4.

    Returns:
        bool: Whether the box was successfully removed
    """

    # Remvoe the box from the world
    self.scene.remove_world_object(self.box_name)

    # We wait for the planning scene to update.
    return self.wait_for_state_update(box_is_attached=False, box_is_known=False, timeout=timeout)
  
  
  def test(self):
    # print("Pose: " + str(self.move_group.get_current_pose()))
    # print("Joint State: " + str(self.move_group.get_current_joint_values()))
    # print(str(self.move_group.get_joints()))
    # print(str(self.move_group.get_end_effector_link()))
    
    # joint_values = self.move_group.get_current_joint_values()
    # for i in range(4, len(joint_values)):
    #   joint_values[i] = 0.5
    #   print("Joint Values " + str(i) + " : " + str(joint_values))
    #   self.move_group.set_joint_value_target(joint_values)
    #   self.move_group.go()
    
    self.go_to_pose(0.4, 0.5, 0.4)
    self.remove_box()
    self.add_box()
    self.move_group.pick(self.box_name)
  