"""Module that contains a MoveItObject"""

import moveit_commander
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseStamped
import random
import rospy


class MoveItObject:
  """Publishes an object to moveit"""

  def __init__(self,
               type,
               model="",
               initial_pose=Pose(),
               world_frame="panda_link0",
               name=""):
    """Initializes the object and publishes it to the topic

    Args:
      type (str): Type of object. See below for supported types.
        "box1" - 0.03 x 0.03 x 0.03 box
      model (str): The filename of the stl.
      initial_pose (geometry_msgs.msg.Pose): The pose of the object relative 
        to the world frame
      world_frame (str): The name of the world frame
      name (str): The name of the object. A random number will be added to make it unique.
    """
    self.type = type
    self.model = model
    self.world_frame = world_frame
    self.scene = moveit_commander.PlanningSceneInterface()

    # Make pose relative to world frame
    self.pose = PoseStamped()
    self.pose.header.frame_id = world_frame
    self.pose.pose = initial_pose

    # Generate a unique name
    # TODO: Should use static count to make name always unique
    self.name = name + str(random.randint(0, 9999))

    self.make_object()

  def make_object(self):
    """Makes the object using the object_type field to determine the type
    """
    if (self.type == "box1"):
      self.__make_box1()
    # TODO: Check if wait for state update is needed

  def __make_box1(self):
    """Makes a 0.03x0.03x0.03 box at the field pose
    """
    self.scene.add_box(self.name, self.pose, size=(0.03, 0.03, 0.03))

  def remove_object(self):
    """Removes the object from the planning scene.
    """
    self.scene.remove_world_object(self.name)
    # TODO: Check if wait for state update is needed

  def set_pose(self, pose):
    """Changes the pose of the object

    Args:
        pose ([type]): [description]
    """
    # TODO: Want a better way to do this
    self.pose.pose = pose
    self.remove_object()
    self.make_object()

  def wait_for_state_update(self,
                            box_is_known=True,
                            box_is_attached=False,
                            timeout=4):
    """Waits until we reach the expected state.

    Args:
      box_is_known (bool): Optional; Expected value for whether the box is in the scene. 
        Defaults to True.
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
      attached_objects = self.scene.get_attached_objects([self.name])
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
