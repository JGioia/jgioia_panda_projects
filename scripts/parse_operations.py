#!/usr/bin/env python
"""Executable module that parses poses and moves the Panda"""

import move_group_interface
import sys
from moveit_commander import robot
import rospy
import time
import movement_pb2


def parse_protobuf(file_path):
  moveCol = movement_pb2.MovementCollection()
  file = open(file_path, "r")
  moveCol.ParseFromString(file.read())
  file.close()
  return moveCol


def run_move_op(moveOp, robotInterface):
  if moveOp.HasField("wait"):
    print("Waiting " + str(moveOp.wait.seconds) + " seconds...")
    time.sleep(moveOp.wait.seconds)

  elif moveOp.HasField("poseGoal"):
    xGoal = None
    yGoal = None
    zGoal = None

    if moveOp.poseGoal.HasField("x"):
      xGoal = moveOp.poseGoal.x
    if moveOp.poseGoal.HasField("y"):
      yGoal = moveOp.poseGoal.y
    if moveOp.poseGoal.HasField("z"):
      zGoal = moveOp.poseGoal.z

    print("Moving to position x = " + str(xGoal) + ", y = " + str(yGoal) +
          ", z = " + str(zGoal) + "...")
    robotInterface.go_to_pose(xGoal, yGoal, zGoal)

  elif moveOp.HasField("jointGoal"):
    jointGoal = [None for i in range(7)]

    # I'm not a big fan of doing it this way, but I want to maintain the ability
    # to just specify some of the joints
    if moveOp.jointGoal.HasField("joint1"):
      jointGoal[0] = moveOp.jointGoal.joint1
    if moveOp.jointGoal.HasField("joint2"):
      jointGoal[1] = moveOp.jointGoal.joint2
    if moveOp.jointGoal.HasField("joint3"):
      jointGoal[2] = moveOp.jointGoal.joint3
    if moveOp.jointGoal.HasField("joint4"):
      jointGoal[3] = moveOp.jointGoal.joint4
    if moveOp.jointGoal.HasField("joint5"):
      jointGoal[4] = moveOp.jointGoal.joint5
    if moveOp.jointGoal.HasField("joint6"):
      jointGoal[5] = moveOp.jointGoal.joint6
    if moveOp.jointGoal.HasField("joint7"):
      jointGoal[6] = moveOp.jointGoal.joint7

    print("Moving to joint goal: " + str(jointGoal) + "...")
    robotInterface.go_to_joint_goal(jointGoal)

  elif moveOp.HasField("gripGoal"):
    if moveOp.gripGoal.HasField("type"):
      if moveOp.gripGoal.type == movement_pb2.GripGoal.GripType.OPEN:
        print("Opening gripper...")
        robotInterface.open_gripper()
      elif moveOp.gripGoal.type == movement_pb2.GripGoal.GripType.CLOSE:
        print("Closing gripper...")
        robotInterface.close_gripper()


def main(file_path):
  """Moves the end effector of the panda to the poses specified in the file given

  Args:
    file_path (string): the file path to the file containing poses
  """

  try:
    # Initializes the move group interface
    robot_interface = move_group_interface.MoveGroupInterface()

    # Parses the poses from the input file path
    moveCol = parse_protobuf(file_path)

    # Goes to every pose in the list of poses
    for moveOp in moveCol.operations:
      run_move_op(moveOp, robot_interface)

  # Ends the program if we get one of these errors
  except rospy.ROSInterruptException:
    return
  except KeyboardInterrupt:
    return


if __name__ == '__main__':
  # Default file we are parsing poses from
  file_path = "/home/joseph/ws_moveit/src/jgioia_panda_projects/proto/message1.txt"

  # Overrides default if file is specified in arguments
  if len(sys.argv) > 1:
    file_path = sys.argv[1]

  main(file_path)
