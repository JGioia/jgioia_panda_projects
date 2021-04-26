#!/usr/bin/env python
"""An executable module where I test features."""

from move_group_interface import MoveGroupInterface
from depth_sensor_interface import DepthSensorInterface
from gazebo_object_importer import LinkPoseInterface
from ar_object_importer import ArObjectImporter
from moveit_commander import robot
from moveit_object import MoveItObject
from geometry_msgs.msg import Pose

import rospy
import time
import numpy as np


def main():
  """Runs testing code."""
  rospy.init_node("Testing")

  robot_interface = MoveGroupInterface()
  depth_interface = DepthSensorInterface()
  link_interface = LinkPoseInterface()

  rospy.sleep(2)

  floor = MoveItObject(type="floor")
  time.sleep(10)
  floor.delete()

  # robot_interface.go_to_pose(yaw=0)
  # robot_interface.test()
  # time.sleep(4)
  # robot_interface.go_to_pose(yaw=np.pi)
  # robot_interface.test()

  # robot_interface.rotate_gripper(0)
  # time.sleep(5)
  # robot_interface.rotate_gripper(-np.pi - 0.01)

  # robot_interface.go_to_joint_goal([None, None, None, None, None, None, -2.89])
  # robot_interface.test()
  # time.sleep(4)
  # robot_interface.go_to_joint_goal([None, None, None, None, None, None, 2.89])
  # robot_interface.test()

  # # # DEMO 4: Object state publisher
  # box_pose = link_interface.find_pose("box_purple1::object_link")
  # box = MoveItObject(type="box1", initial_pose=box_pose)
  # while (True):
  #   time.sleep(0.1)
  #   box.set_pose(link_interface.find_pose("box_purple1::object_link"))

  # robot_interface.slide_to(0.5, -0.3)
  # robot_interface.test()

  # DEMO 5: Depth based grabber

  # # DEMO 6: Table
  # table = MoveItObject(
  #     type="mesh",
  #     mesh="/home/joseph/ws_moveit/src/jgioia_panda_projects/models/table.STL")

  # DEMO 7: Collision avoidance of a gazebo object
  # box_pose = link_interface.find_pose("box::link")
  # box = MoveItObject(type="box1", initial_pose=box_pose)
  # robot_interface.go_to_pose(box_pose.position.x, box_pose.position.y - 0.1,
  #                            0.1 + box_pose.position.z)
  # robot_interface.close_gripper()
  # robot_interface.go_to_pose(box_pose.position.x, box_pose.position.y + 0.1,
  #                            0.1 + box_pose.position.z)
  # box.delete()
  # robot_interface.go_to_pose(box_pose.position.x, box_pose.position.y - 0.1,
  #                            0.1 + box_pose.position.z)

  # # DEMO 8: MoveIt object publishing from AR tag
  # object_importer = ArObjectImporter(MoveItObject(type="box1"))
  # time.sleep(30)
  # object_importer.stop()


def demo_archive():
  """Archive for old demos. Uncomment out anything you want to test.
  """
  robot_interface = MoveGroupInterface()
  depth_interface = DepthSensorInterface("camera/depth/image_raw")
  link_interface = LinkPoseInterface("/gazebo/link_states")

  time.sleep(1)

  # # DEMO 1: Almost pick up
  # # Box should be at 0.4 -0.5 ~0
  # print("open")
  # robot_interface.open_gripper()
  # print("move")
  # robot_interface.go_to_pose(0.41, -0.5, 0.2)
  # print("move")
  # robot_interface.go_to_pose(0.41, -0.5, 0.11)
  # print("close")
  # robot_interface.go_to_hand_joint_goal([0.014, 0.014])
  # print("move")
  # robot_interface.go_to_pose(0.41, -0.5, 0.2)

  # # # DEMO 2: Link state based grabber
  # print(link_interface.find_pose("box::link"))
  # robot_interface.box_pose = link_interface.find_pose("box::link")
  # robot_interface.grab_box()

  # # DEMO 3: Collision avoidance
  # wall_pose = Pose()
  # wall_pose.position.x = 1.5
  # wall = MoveItObject(type="wall", initial_pose=wall_pose)
  # robot_interface.go_to_pose(0.7, 0.3, 0.3)
  # robot_interface.go_to_pose(0.7, -0.3, 0.3)
  # wall.delete()

  # # DEMO 4: Object state publisher
  # box_pose = link_interface.find_pose("box::link")
  # box = MoveItObject(type="box1", initial_pose=box_pose)
  # while (True):
  #   time.sleep(0.1)
  #   box.set_pose(link_interface.find_pose("box::link"))


if __name__ == '__main__':
  main()
