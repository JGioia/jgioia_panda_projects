#!/usr/bin/env python
import movement_pb2


def main():
  moveCol = movement_pb2.MovementCollection()
  moveOp = moveCol.operations.add()
  moveOp.type = movement_pb2.MovementType.WAIT
  moveOp.wait.seconds = 2.0

  print(moveOp.wait.SerializeToString())

  f = open("message1.proto", "wb")
  f.write(moveCol.SerializeToString())
  f.close()


if __name__ == '__main__':
  main()
