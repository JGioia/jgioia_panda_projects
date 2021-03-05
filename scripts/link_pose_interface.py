"""Module that contains the LinkPoseInterface class"""

import rospy
import gazebo_msgs


class LinkPoseInterface():
  """Provides useful methods for getting link poses from gazebo"""

  def __init__(self, topic_name):
    """Initializes the interface to retrieve link poses from gazebo

    Args:
      topic_name (str): The name of the gazebo link pose topic. The topic
        return a gazebo_msgs.msg.LinkStates
    """
    rospy.init_node('LinkPoseInterface')
    rospy.Subscriber(topic_name, gazebo_msgs.msg.LinkStates, self.update_states)

  def update_states(self, link_states):
    """Updates the internal representation of the link states
    
    Args:
      link_state (gazebo_msgs.msg.LinkStates): The new link states
    """
    self.link_states = link_states

  def find_pose(self, link_name):
    """Returns the pose of the specified link

    Args:
      link_name (str): The name of the link
    """
    pass
    # TODO: Make this work