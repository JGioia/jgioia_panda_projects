"""Module that contains the LinkPoseInterface class"""

import rospy
from gazebo_msgs.msg import LinkStates
from geometry_msgs.msg import Pose

import time


class GazeboObjectImporter():
  """Updates the position of a MoveItObject based on the pose of a link in gazebo"""

  def __init__(self, moveit_object, link_name):
    """Starts the GazeboObjectImporter. 

    Args:
      moveit_object (MoveItObject): The object whose pose we want to update
      link_name (str): The name of the link whose pose we are using to update 
        the object. Should be in form "model::link".
    """
    self.link_name = link_name
    self.moveit_object = moveit_object
    self.moveit_object.delete()
    self.is_stopped = False
    self.is_visible = False
    self.last_pose = Pose()
    self.link_pose_interface = LinkPoseInterface(launch_subscriber=False)

    rospy.Subscriber("/gazebo/link_states", LinkStates, self.__update)

  def __update(self, link_states):
    """Updates the position of the object based on the position of the link.

    Args:
      link_states (gazebo_msgs.msg.LinkStates): The new link states
    """
    if (not self.is_stopped):
      self.ticks = 1
      self.link_pose_interface.link_states = link_states
      new_pose = self.link_pose_interface.find_pose(self.link_name)
      if (new_pose != None):
        # Doesn't work like it should
        if (self.last_pose != new_pose or self.is_visible):
          self.last_pose = new_pose
          self.is_visible = True
          self.moveit_object.set_pose(new_pose)
      elif (self.is_visible):
        self.is_visible = False
        self.moveit_object.delete()

  def update_pose(self):
    """Updates the pose of the object stored
    """

  def stop(self):
    """Stops updating and removes the object"""
    self.is_stopped = True
    if (not self.is_visible):
      self.moveit_object.delete()
      self.is_visible = False


class LinkPoseInterface():
  """Provides useful methods for getting link poses from gazebo"""

  def __init__(self, launch_subscriber=True):
    """Initializes the interface to retrieve link poses from gazebo"""
    self.link_states = LinkStates()

    if (launch_subscriber):
      rospy.Subscriber("/gazebo/link_states", LinkStates, self.__update)

  def __update(self, link_states):
    """Updates the internal representation of the link states
    
    Args:
      link_state (gazebo_msgs.msg.LinkStates): The new link states
    """
    self.link_states = link_states

  def find_pose(self, link_name):
    """Returns the pose of the specified link

    Args:
      link_name (str): The name of the link in the form 'model::link'
      
    Returns: geometry_msgs.msg.Pose of the specified link
    """
    try:
      index = self.link_states.name.index(link_name)
    except ValueError:
      return None

    current_pose = self.link_states.pose[index]
    return current_pose

  def get_link_names(self):
    """Returns a list of link names

    Returns: list(str) of the link names
    """
    return self.link_states.name
