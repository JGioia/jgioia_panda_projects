"""Module that contains the ArObjectImporter"""

from ar_track_alvar_msgs.msg import AlvarMarkers
from geometry_msgs.msg import Pose
import rospy


class ArObjectImporter():
  """Updates the position of a MoveItObject based on the pose of an AR tag"""

  def __init__(self, moveit_object):
    """Starts the ArObjectImporter. Note: uses the position any of the tags found.

    Args:
      moveit_object (MoveItObject): The object whose pose we want to update.
    """
    self.moveit_object = moveit_object
    self.moveit_object.delete()
    self.is_stopped = False
    self.is_visible = False
    self.max_ticks_since_seen = 10
    self.ticks_since_seen = self.max_ticks_since_seen + 1
    self.last_pose = Pose()

    rospy.Subscriber("/ar_pose_marker", AlvarMarkers, self.__update)

  def __update(self, alvar_markers):
    """Updates the position and visibility of the object using the first marker

    Args:
      alvar_markers (ar_track_alvar_msgs.msg.AlvarMarkers): The list of markers to use
    """
    if (not self.is_stopped):
      if (len(alvar_markers.markers) > 0):
        self.ticks_since_seen = 0
        self.is_visible = True
        self.last_pose = alvar_markers.markers[0].pose.pose
        self.moveit_object.set_pose(self.last_pose)
      else:
        self.ticks_since_seen += 1
        if (self.ticks_since_seen > self.max_ticks_since_seen and
            self.is_visible):
          self.moveit_object.delete()
          self.is_visible = False

  def stop(self):
    """Stops updating and removes the object"""
    self.is_stopped = True
    if (not self.is_visible):
      self.moveit_object.delete()
      self.is_visible = False
