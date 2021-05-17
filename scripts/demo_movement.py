#!/usr/bin/env python
"""An executable module that tests basic movement and object functionalities"""

from move_group_interface import MoveGroupInterface
from moveit_object import MoveItObject
from geometry_msgs.msg import Pose
import rospy


def main():
  """Runs the movement demo"""
  rospy.init_node("Movement_Demo")
  moveGroupInterface = MoveGroupInterface()

  print("Testing arm joint movement")
  moveGroupInterface.go_to_joint_goal([1, None, None, None, None, None, 1])
  rospy.sleep(1)

  print("Testing hand joint movement")
  moveGroupInterface.go_to_hand_joint_goal([0.02, 0.02])
  rospy.sleep(1)

  print("Testing pose goal movement")
  moveGroupInterface.go_to_pose(0.3, 0.5, 0.3)
  rospy.sleep(1)

  print("Testing preset gripper positions")
  moveGroupInterface.open_gripper()
  rospy.sleep(1)
  moveGroupInterface.close_gripper()
  rospy.sleep(1)

  print("Testing slide movement at current z position")
  moveGroupInterface.slide_to(0.5, 0.3)
  rospy.sleep(1)

  print("Testing gripper rotation")
  moveGroupInterface.rotate_gripper(1.753)
  rospy.sleep(1)

  print("Testing object avoidance")
  moveGroupInterface.go_to_pose(0.3, 0.5, 0.3)
  wall = MoveItObject(type="wall")
  moveGroupInterface.go_to_pose(0.3, -0.5, 0.3)
  rospy.sleep(1)
  wall.delete()
  rospy.sleep(1)

  print("Testing grab object")
  box_pose = Pose()
  box_pose.position.x = 0.4
  box_pose.position.y = 0.4
  box_pose.position.z = 0.1
  box = MoveItObject(type="box1", initial_pose=box_pose)
  rospy.sleep(1)
  moveGroupInterface.grab_object(box)
  rospy.sleep(1)
  box.delete()


if __name__ == '__main__':
  main()
