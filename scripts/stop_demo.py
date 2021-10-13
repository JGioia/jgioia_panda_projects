#!/usr/bin/env python
"""An executable module to test stopping an action in progress"""

from move_group_interface import MoveGroupInterface

import rospy

def main():
  rospy.init_node("stop_demo")
  robot_interface = MoveGroupInterface()
  input("Enter a number")
  robot_interface.stop()

if __name__ == '__main__':
  main()
