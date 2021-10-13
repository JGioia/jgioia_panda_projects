"""Module that contains a MoveItObject"""

import moveit_commander
from move_group_interface import MoveGroupInterface
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseStamped
import tf2_ros
import tf2_geometry_msgs
import random
import rospy


class MoveItObject:
  """An object that has a pose, can be grabbed, and can be added to the planning scene (for collision avoidance)."""

  def __init__(self,
               type,
               mesh="",
               initial_pose=Pose(),
               world_frame="panda_link0",
               name="",
               collision=True):
    """Initializes the object and publishes it to the topic

    Args:
      type (str): Type of object. See below for supported types. Feel free to add your own.
        "box1" - 0.03 x 0.03 x 0.03 box
        "wall" - 2 x 0.25 x 2 box
        "floor" - 3 x 3 x 0.1 box that defaults to be centered right below the robot
        "mesh" - Uses a file to generated the object
      model (str): The filename of the mesh.
      initial_pose (geometry_msgs.msg.Pose): The pose of the object relative 
        to the world frame
      world_frame (str): The name of the world frame
      name (str): The name of the object. A random number will be added to make it unique.
      collision (bool): Determines whether the object will be added to the planning scene
    """
    self.type = type
    self.mesh = mesh
    self.world_frame = world_frame
    self.collision = collision
    self.move_group_interface = MoveGroupInterface()
    self.scene = moveit_commander.PlanningSceneInterface(synchronous=True)

    # Default pose is different for some types
    if (type == "floor" and initial_pose == Pose()):
      initial_pose.position.z = 0.1
    elif (type == "wall" and initial_pose == Pose()):
      initial_pose.position.x = 1.3

    # Make pose relative to world frame
    self.pose = PoseStamped()
    self.pose.header.frame_id = world_frame
    self.pose.pose = initial_pose

    self.grasp_offset = Pose()
    self.grasp_width = 0

    self.tfBuffer = tf2_ros.Buffer()
    self.listener = tf2_ros.TransformListener(self.tfBuffer)

    # Generate a unique name
    # TODO: Maybe use static count to make name always unique
    self.name = name + str(random.randint(0, 9999))

    self.visibility = False
    self.make()

  def make(self):
    """Makes the object using the object_type field to determine the type
    """
    if (self.type == "box1"):
      self.__make_box1()
    elif (self.type == "wall"):
      self.__make_wall()
    elif (self.type == "mesh"):
      self.__make_mesh()
    elif (self.type == "floor"):
      self.__make_floor()
    if (self.collision):
      self.visibility = True

  def __make_box1(self):
    """Defines grasp position and adds a 0.03 x 0.03 x 0.03 box if collision is on.
    """
    self.grasp_offset.position.z = 0.1
    self.grasp_width = 0.0306
    if (self.collision):
      self.scene.add_box(self.name, self.pose, size=(0.03, 0.03, 0.03))

  def __make_wall(self):
    """Defines grasp position and adds a 2 x 0.25 x 2 box if collision is on
    """
    self.grasp_offset = None
    self.grasp_width = None
    if (self.collision):
      self.scene.add_box(self.name, self.pose, size=(2, 0.25, 2))

  def __make_floor(self):
    """Defines grasp position and add a floor if collision is on
    """
    # TODO: Make this behavior more standard
    self.grasp_offset.position.z = None
    self.grasp_width = None
    if (self.collision):
      self.pose.pose.position.x = 0.6
      self.scene.add_box(self.name, self.pose, size=(1, 2, 0.2))
      self.pose.pose.position.x = -0.7
      self.scene.add_box(self.name + "1", self.pose, size=(1, 2, 0.2))
      self.pose.pose.position.x = 0
      self.pose.pose.position.y = 0.6
      self.scene.add_box(self.name + "2", self.pose, size=(1, 1, 0.2))
      self.pose.pose.position.y = -0.6
      self.scene.add_box(self.name + "3", self.pose, size=(1, 1, 0.2))

  def __make_mesh(self):
    """Adds an object specified by the stl at the file of the name specified by the model field
    """
    self.grasp_offset.position.z = None
    self.grasp_width = None
    if (self.collision):
      self.scene.add_mesh(self.name, self.pose, self.mesh, size=(0.1, 0.1, 0.1))

  def delete(self):
    """Removes the object from the planning scene.
    """
    if (self.visibility):
      self.scene.remove_world_object(self.name)
      if (self.type == "floor"):
        self.scene.remove_world_object(self.name + "1")
        self.scene.remove_world_object(self.name + "2")
        self.scene.remove_world_object(self.name + "3")
      self.visibility = False

  def set_pose(self, pose):
    """Changes the pose of the object

    Args:
      pose (geometry_msgs.msg.Pose): The new pose for the object
    """
    self.pose.pose = pose
    self.delete()
    self.make()

  def set_pose_stamped(self, pose):
    """Changes the pose of the object

    Args:
      pose (geometry_msgs.msg.PoseStamped): The new pose in the given reference frame
    """
    transform = self.tfBuffer.lookup_transform(self.world_frame,
                                               pose.header.frame_id,
                                               rospy.Time())

    self.pose = tf2_geometry_msgs.do_transform_pose(pose, transform)
    self.delete()
    self.make()

  def check_collision(self):
    """ Checks whether this object is in the operating zone of the robot and stops the robot if so.

    Should be called by a subscription to a channel that published object location. 
    Should change method when expected behavior is better defined.

    Returns:
      A bool that represents whether a collision was detected
    """
    if (self.visibility and self.collision):
      x = self.pose.pose.position.x
      y = self.pose.pose.position.y
      z = self.pose.pose.position.z
      if (x >= -2 and x <= 2 and y >=-2 and y <= 2 and z <=-2 and z >= 2):
        self.move_group_interface.stop()


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
