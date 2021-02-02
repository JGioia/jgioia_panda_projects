#!/usr/bin/env python
import movement_pb2
"""Creates the protocol buffer that contains the move operations."""


def main():
  """Creates the protocol buffer that contains the move operations.
  """
  move_col = movement_pb2.MovementCollection()

  # Move
  move_op1 = move_col.operations.add()
  move_op1.poseGoal.x = 0.4
  move_op1.poseGoal.y = 0.4
  move_op1.poseGoal.z = 0.4

  # Add box
  move_op2 = move_col.operations.add()
  move_op2.boxControl.type = movement_pb2.BoxControl.ControlType.ADD
  move_op2.boxControl.x = 0.4
  move_op2.boxControl.y = -0.5
  move_op2.boxControl.z = 0.03

  # Grab
  move_op3 = move_col.operations.add()
  move_op3.manipulateBox.type = movement_pb2.ManipulateBox.ManipulationType.GRAB

  # Move
  move_op4 = move_col.operations.add()
  move_op4.poseGoal.x = -0.4
  move_op4.poseGoal.y = -0.5
  move_op4.poseGoal.z = 0.13

  # Detach
  move_op5 = move_col.operations.add()
  move_op5.boxControl.type = movement_pb2.BoxControl.ControlType.DETACH

  # Open Gripper
  move_op6 = move_col.operations.add()
  move_op6.gripGoal.type = movement_pb2.GripGoal.GripType.OPEN

  # Move
  move_op7 = move_col.operations.add()
  move_op7.poseGoal.x = 0.4
  move_op7.poseGoal.y = -0.5
  move_op7.poseGoal.z = 0.3

  # Remove
  move_op8 = move_col.operations.add()
  move_op8.boxControl.type = movement_pb2.BoxControl.ControlType.REMOVE

  f = open("../proto/demo2.txt", "wb")
  f.write(move_col.SerializeToString())
  f.close()


if __name__ == '__main__':
  main()
