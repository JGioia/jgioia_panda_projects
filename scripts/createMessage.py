#!/usr/bin/env python
import movement_pb2


def main():
  moveCol = movement_pb2.MovementCollection()

  moveOp1 = moveCol.operations.add()
  moveOp1.poseGoal.x = 0.4
  moveOp1.poseGoal.z = 0.3

  moveOp2 = moveCol.operations.add()
  moveOp2.wait.seconds = 1.0

  moveOp3 = moveCol.operations.add()
  moveOp3.jointGoal.joint1 = 0.5

  moveOp4 = moveCol.operations.add()
  moveOp4.wait.seconds = 0.1

  f = open("../proto/message1.txt", "wb")
  f.write(moveCol.SerializeToString())
  f.close()


if __name__ == '__main__':
  main()
