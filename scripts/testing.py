#!/usr/bin/env python
"""An executable module where I test features. Currently working on pick and place."""

import move_group_interface
import depth_sensor_interface
import rospy
import time


def main():
  """Runs testing code."""
  try:
    # robot_interface = move_group_interface.MoveGroupInterface()
    # robot_interface.test()
    depth_interface = depth_sensor_interface.DepthSensorInterface(
        "camera/depth/image_raw")
    time.sleep(5)
    print(depth_interface.depth_image)
  except rospy.ROSInterruptException:
    return
  except KeyboardInterrupt:
    return


if __name__ == '__main__':
  main()
