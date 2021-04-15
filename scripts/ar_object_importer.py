"""Module that contains the ArObjectImporter"""

from ar_track_alvar_msgs.msg import AlvarMarkers
from move_group_interface import MoveGroupInterface
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseStamped
from moveit_commander import move_group
import rospy
import time


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
    self.ticks = 0
    self.ticks_seen = 0
    self.ticks_since_seen = self.max_ticks_since_seen + 1
    self.last_pose = PoseStamped()
    self.move_group_interface = MoveGroupInterface()

    rospy.Subscriber("/ar_pose_marker", AlvarMarkers, self.__update)

  def __update(self, alvar_markers):
    """Updates the position and visibility of the object using the first marker

    Args:
      alvar_markers (ar_track_alvar_msgs.msg.AlvarMarkers): The list of markers to use
    """
    if (not self.is_stopped):
      self.ticks += 1
      if (len(alvar_markers.markers) > 0):
        self.ticks_seen += 1
        self.is_visible = True
        self.last_pose = alvar_markers.markers[0].pose
        self.last_pose.header.frame_id = "camera_color_optical_frame"
        self.moveit_object.set_pose_stamped(self.last_pose)
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

  def scan(self):
    """ Moves the robot to different positions to look for an ar tag.
    
    Stops scanning when it has completed a cycle or it detects the tag for
    at least 80% of the time at one of its stops.
    
    Returns: True if it detects a tag and False if not
    """
    poses = [[0.4, -0.5, 0.3], [0, -0.5, 0.3], [-0.4, -0.5, 0.3]]
    threshold = 0.8
    for pose in poses:
      self.move_group_interface.go_to_pose(x=pose[0], y=pose[1], z=pose[2])
      self.ticks = 0
      self.ticks_seen = 0
      time.sleep(1)
      print("proportion of time seen: " + str(self.ticks_seen / self.ticks))
      if (self.ticks_seen / self.ticks > threshold):
        return True
    return False
