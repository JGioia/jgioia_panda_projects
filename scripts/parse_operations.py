#!/usr/bin/env python
"""Executable module that parses operations and moves the Panda"""

import move_group_interface
import sys
import rospy
import time
import movement_pb2


def parse_protobuf(file_path):
  """Parses the MovementCollection stored as a str in the given file

  Args:
      file_path (str): The path to the file storing the MovementCollection

  Returns:
      The MovementCollection parsed from the file
  """
  move_col = movement_pb2.MovementCollection()
  file = open(file_path, "r")
  move_col.ParseFromString(file.read())
  file.close()
  return move_col


def run_move_op(move_op, robot_interface):
  """Makes the robot do the given MovementOperation

  Args:
      move_op (MovementOperation): The operation to do
      robot_interface (MoveGroupInterface): The interface for the robot to 
        run the operations on
  """
  if move_op.HasField("wait"):
    print("Waiting " + str(move_op.wait.seconds) + " seconds...")
    time.sleep(move_op.wait.seconds)

  elif move_op.HasField("poseGoal"):
    x_goal = None
    y_goal = None
    z_goal = None

    if move_op.poseGoal.HasField("x"):
      x_goal = move_op.poseGoal.x
    if move_op.poseGoal.HasField("y"):
      y_goal = move_op.poseGoal.y
    if move_op.poseGoal.HasField("z"):
      z_goal = move_op.poseGoal.z

    print("Moving to position x = " + str(x_goal) + ", y = " + str(y_goal) +
          ", z = " + str(z_goal) + "...")
    robot_interface.go_to_pose(x_goal, y_goal, z_goal)

  elif move_op.HasField("jointGoal"):
    joint_goal = [None for i in range(7)]

    # I'm not a big fan of doing it this way, but I want to maintain the ability
    # to just specify some of the joints
    if move_op.jointGoal.HasField("joint1"):
      joint_goal[0] = move_op.jointGoal.joint1
    if move_op.jointGoal.HasField("joint2"):
      joint_goal[1] = move_op.jointGoal.joint2
    if move_op.jointGoal.HasField("joint3"):
      joint_goal[2] = move_op.jointGoal.joint3
    if move_op.jointGoal.HasField("joint4"):
      joint_goal[3] = move_op.jointGoal.joint4
    if move_op.jointGoal.HasField("joint5"):
      joint_goal[4] = move_op.jointGoal.joint5
    if move_op.jointGoal.HasField("joint6"):
      joint_goal[5] = move_op.jointGoal.joint6
    if move_op.jointGoal.HasField("joint7"):
      joint_goal[6] = move_op.jointGoal.joint7

    print("Moving to joint goal: " + str(joint_goal) + "...")
    robot_interface.go_to_joint_goal(joint_goal)

  elif move_op.HasField("gripGoal"):
    if move_op.gripGoal.HasField("type"):
      if move_op.gripGoal.type == movement_pb2.GripGoal.GripType.OPEN:
        print("Opening gripper...")
        robot_interface.open_gripper()
      elif move_op.gripGoal.type == movement_pb2.GripGoal.GripType.CLOSE:
        print("Closing gripper...")
        robot_interface.close_gripper()

  elif move_op.HasField("boxControl"):
    if move_op.boxControl.HasField("type"):
      if move_op.boxControl.type == movement_pb2.BoxControl.ControlType.ADD:
        if move_op.boxControl.HasField("x") and move_op.boxControl.HasField(
            "y") and move_op.boxControl.HasField("z"):
          print("Adding box at x = " + str(move_op.boxControl.x) + ", y = " +
                str(move_op.boxControl.y) + ", z = " +
                str(move_op.boxControl.z) + "...")
          robot_interface.add_box2(move_op.boxControl.x, move_op.boxControl.y,
                                   move_op.boxControl.z)
      elif move_op.boxControl.type == movement_pb2.BoxControl.ControlType.REMOVE:
        print("Removing box...")
        robot_interface.remove_box()
      elif move_op.boxControl.type == movement_pb2.BoxControl.ControlType.ATTACH:
        print("Attaching box...")
        robot_interface.attach_box()
      elif move_op.boxControl.type == movement_pb2.BoxControl.ControlType.DETACH:
        print("Detaching box...")
        robot_interface.detach_box()

  elif move_op.HasField("manipulateBox"):
    if move_op.manipulateBox.HasField("type"):
      if move_op.manipulateBox.type == movement_pb2.ManipulateBox.ManipulationType.GRAB:
        print("Grabbing box...")
        robot_interface.grab_box()


def main(file_path):
  """Moves the end effector of the panda to the poses specified in the file given

  Args:
    file_path (string): the file path to the file containing poses
  """
  try:
    # Initializes the move group interface
    robot_interface = move_group_interface.MoveGroupInterface()

    # Parses the poses from the input file path
    move_col = parse_protobuf(file_path)

    # Print a line to make the debug messages easier to read
    print
    print("Running Operations:")

    # Goes to every pose in the list of poses
    for move_op in move_col.operations:
      run_move_op(move_op, robot_interface)

  # Ends the program if we get one of these errors
  except rospy.ROSInterruptException:
    return
  except KeyboardInterrupt:
    return


if __name__ == '__main__':
  # Default file we are parsing poses from
  file_path = "/home/joseph/ws_moveit/src/jgioia_panda_projects/proto/demo2.txt"

  # Overrides default if file is specified in arguments
  if len(sys.argv) > 1:
    file_path = sys.argv[1]

  main(file_path)
