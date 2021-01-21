#!/usr/bin/env python
import movement_pb2
"""Creates the protocol buffer that contains the move operations."""


def main():
  """Creates the protocol buffer that contains the move operations.
  """
  move_col = movement_pb2.MovementCollection()

  move_op1 = move_col.operations.add()
  move_op1.poseGoal.x = 0.4
  move_op1.poseGoal.z = 0.3

  move_op2 = move_col.operations.add()
  move_op2.wait.seconds = 1.0

  move_op3 = move_col.operations.add()
  move_op3.jointGoal.joint1 = 0.5

  move_op4 = move_col.operations.add()
  move_op4.wait.seconds = 0.1

  move_op5 = move_col.operations.add()
  move_op5.gripGoal.type = movement_pb2.GripGoal.GripType.OPEN

  move_op6 = move_col.operations.add()
  move_op6.gripGoal.type = movement_pb2.GripGoal.GripType.CLOSE

  f = open("../proto/message1.txt", "wb")
  f.write(move_col.SerializeToString())
  f.close()


if __name__ == '__main__':
  main()
