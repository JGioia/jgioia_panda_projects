#!/usr/bin/env python
""" An executable module where I test features. Currently working on pick and place."""

import move_group_interface
import rospy


def main():
  """ Runs testing code """
  try:
    robot_interface = move_group_interface.MoveGroupInterface()
    robot_interface.test()
  except rospy.ROSInterruptException:
    return
  except KeyboardInterrupt:
    return


if __name__ == '__main__':
  main()
