#!/usr/bin/env python
"""An executable module where I test features. Currently working on pick and place."""

from move_group_interface import MoveGroupInterface
from depth_sensor_interface import DepthSensorInterface
from link_pose_interface import LinkPoseInterface
from moveit_object import MoveItObject
from geometry_msgs.msg import Pose

import rospy
import time


def main():
  """Runs testing code."""
  try:
    rospy.init_node("Testing")

    robot_interface = MoveGroupInterface()
    depth_interface = DepthSensorInterface("camera/depth/image_raw")
    link_interface = LinkPoseInterface("/gazebo/link_states")

    time.sleep(1)

    # # DEMO 1: Almost pick up
    # print(depth_interface.depth_image)
    # robot_interface.test()

    # # DEMO 2: Link state based grabber
    # print(link_interface.find_pose("box::link"))
    # robot_interface.box_pose = link_interface.find_pose("box::link")
    # robot_interface.grab_box()

    # DEMO 3: Collision avoidance

    # # DEMO 4: Object state publisher
    # box_pose = link_interface.find_pose("box::link")
    # box = MoveItObject(type="box1", initial_pose=box_pose)
    # while (True):
    #   time.sleep(0.1)
    #   box.set_pose(link_interface.find_pose("box::link"))

    # DEMO 5: Depth based grabber

    # # TEST 1: Print link names
    # print(link_interface.get_link_names())


if __name__ == '__main__':
  main()
