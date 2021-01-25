#!/usr/bin/env python
"""Runs a very simple pick and place script."""

import move_group_interface
import rospy


def main():
  """Runs the pick and place demo."""
  robot_interface = move_group_interface.MoveGroupInterface()

  print("going to pose")
  robot_interface.go_to_pose(0.4, -0.5, 0.3)

  print("adding box")
  robot_interface.add_box()

  print("closing gripper")
  robot_interface.go_to_hand_joint_goal([0.017, 0])

  print("moving to place pose")
  robot_interface.attach_box()
  robot_interface.go_to_pose(0.4, 0.5, 0.3)

  print("opening gripper")
  robot_interface.detach_box()
  robot_interface.open_gripper()

  print("moving to post-place pose")
  robot_interface.go_to_pose(0.4, -0.5, 0.3)

  print("removing box")
  robot_interface.remove_box()


if __name__ == '__main__':
  main()