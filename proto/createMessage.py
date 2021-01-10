#!/usr/bin/env python
import movement_pb2
import sys


def main():
  moveCol = movement_pb2.MovementCollection()
  moveOp = moveCol.operations.add()
  moveOp.type = movement_pb2.MovementType.WAIT
  moveOp.wait.seconds = 2.0

  f = open("message1.txt", "wb")
  f.write(moveCol.SerializeToString())
  f.close()


if __name__ == '__main__':
  main()
