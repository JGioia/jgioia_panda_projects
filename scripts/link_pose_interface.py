"""Module that contains the LinkPoseInterface class"""

import rospy
from gazebo_msgs.msg import LinkStates
from collections import defaultdict
from geometry_msgs.msg import Pose


class LinkPoseInterface():
  """Provides useful methods for getting link poses from gazebo"""

  def __init__(self, topic_name):
    """Initializes the interface to retrieve link poses from gazebo

    Args:
      topic_name (str): The name of the gazebo link pose topic. The topic
        return a gazebo_msgs.msg.LinkStates
    """
    self.link_states = LinkStates()

    self.initial_link_states = defaultdict(Pose)
    self.initial_link_states["box::object_link"].position.x = 0.4
    self.initial_link_states["box::object_link"].position.y = -0.5
    self.initial_link_states["box::object_link"].position.z = 0.4

    rospy.Subscriber(topic_name, LinkStates, self.update_states)

  def update_states(self, link_states):
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
    index = self.link_states.name.index(link_name)
    current_pose = self.link_states.pose[index]
    initial_pose = self.initial_link_states[link_name]

    adjusted_pose = Pose()
    adjusted_pose.position.x = current_pose.position.x + initial_pose.position.x
    adjusted_pose.position.y = current_pose.position.y + initial_pose.position.y
    adjusted_pose.position.z = current_pose.position.z + initial_pose.position.z

    return adjusted_pose

  def get_link_names(self):
    """Returns a list of link names

    Returns: list(str) of the link names
    """
    return self.link_states.name
