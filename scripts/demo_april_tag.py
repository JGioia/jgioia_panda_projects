#!/usr/bin/env python
"""An executable module that tries to grab an object detected by april tags"""

from move_group_interface import MoveGroupInterface
from ar_object_importer import ArObjectImporter
from moveit_object import MoveItObject

import rospy
import time


def main():
  """Runs demo"""
  rospy.init_node("April_Tag_Demo")
  robot_interface = MoveGroupInterface()
  time.sleep(1)

  box = MoveItObject(type="box1")
  object_importer = ArObjectImporter(box)
  time.sleep(5)

  print("grabbing")
  robot_interface.grab_box1(box)


if __name__ == '__main__':
  main()
