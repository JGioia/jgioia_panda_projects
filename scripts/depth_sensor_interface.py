"""Module that contains the DepthSensorInterface class. NOTE: This currently does nothing."""

import rospy
from sensor_msgs.msg import Image


class DepthSensorInterface():
  """Provides useful methods for getting information from a depth sensor"""

  def __init__(self):
    """Initializes the interface to get depth info from the specified topic.

    Args:
      topic_name (str): The name of the depth image topic. The topic must 
        return a sensor_msgs.msg.Image.
    """
    self.depth_image = Image()
    rospy.Subscriber("camera/depth/image_raw", Image, self.update_image)

  def update_image(self, depth_image):
    """Updates the internal representation of the depth image.

    Args:
      depth_image (sensor_msgs.msg.Image): The new depth image
    """
    self.depth_image = depth_image

  def detect_objects(self):
    """Detects objects in the last depth image received
    
    Returns:
      A list of poses of the objects detected.
    """
    return []
