#!/usr/bin/env python
"""An executable module that preforms the gazebo manipulation demo."""

from move_group_interface import MoveGroupInterface
from depth_sensor_interface import DepthSensorInterface
from gazebo_object_importer import LinkPoseInterface
from ar_object_importer import ArObjectImporter
from moveit_commander import robot
from moveit_object import MoveItObject
from geometry_msgs.msg import Pose
from gazebo_object_importer import GazeboObjectImporter

import rospy
import time
import numpy as np


def main():
  """Sorts the boxes by color"""

  # NOTE: I hardcoded everything instead of using my prebuilt methods
  # for grabbing objects, because I was trying to tweak all of the parameters
  # to get it to work consistently. This did not work :(

  rospy.init_node("Gazebo_Manipulation_Demo")

  robot_interface = MoveGroupInterface()

  time.sleep(1)

  floor = MoveItObject(type="floor")

  robot_interface.open_gripper()

  # Purple box 1
  robot_interface.go_to_pose(0.2, 0.6, 0.2)
  robot_interface.go_to_pose(0.2, 0.6, 0.125)
  robot_interface.go_to_hand_joint_goal([0.01505, 0.01505])
  robot_interface.slide_to(0.2, 0.5)
  robot_interface.slide_to(0.5, 0.5)
  robot_interface.open_gripper()
  robot_interface.go_to_pose(0.5, 0.5, 0.2)

  # Purple box 2
  robot_interface.go_to_pose(-0.1, 0.601, 0.2)
  robot_interface.go_to_pose(-0.1, 0.601, 0.125)
  robot_interface.go_to_hand_joint_goal([0.01505, 0.01505])
  robot_interface.slide_to(-0.1, 0.5)
  robot_interface.slide_to(-0.11, 0.5, False)
  robot_interface.slide_to(0.4, 0.5)
  robot_interface.open_gripper()
  robot_interface.go_to_pose(0.4, 0.5, 0.2)

  # Yellow box 1
  robot_interface.go_to_pose(0.1, 0.6, 0.2)
  robot_interface.go_to_pose(0.1, 0.6, 0.125)
  robot_interface.go_to_hand_joint_goal([0.01505, 0.01505])
  robot_interface.slide_to(0.1, 0.5)
  robot_interface.slide_to(-0.5, 0.5)
  robot_interface.open_gripper()
  robot_interface.go_to_pose(-0.5, 0.5, 0.2)

  # Yellow box 2
  robot_interface.go_to_pose(-0.2, 0.6, 0.2)
  robot_interface.go_to_pose(-0.2, 0.6, 0.125)
  robot_interface.go_to_hand_joint_goal([0.01505, 0.01505])
  robot_interface.slide_to(-0.2, 0.5)
  robot_interface.slide_to(-0.21, 0.5, False)
  robot_interface.slide_to(-0.4, 0.5)
  robot_interface.open_gripper()
  robot_interface.go_to_pose(-0.4, 0.5, 0.2)

  # robot_interface.grab_object(purple_box1)
  # robot_interface.slide_to(purple_box1.pose.pose.x,
  #                          purple_box1.pose.pose.y - 0.1)
  # robot_interface.slide_to(0.5, purple_box1.pose.pose.y)

  # yellow_box1_tracker.stop()
  # yellow_box2_tracker.stop()
  # purple_box1_tracker.stop()
  # purple_box2_tracker.stop()
  # yellow_box1.delete()
  # yellow_box2.delete()
  # purple_box1.delete()
  # purple_box2.delete()
  floor.delete()


if __name__ == '__main__':
  main()
