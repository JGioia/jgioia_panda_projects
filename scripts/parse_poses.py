#!/usr/bin/env python
""" Executable module that parses poses and moves the Panda """

import move_group_interface
import sys
import rospy
import time


def parse_csv(file_path):
  """
  Parses a file containing comma seperated triples of floats representing pose goals
  
  Args:
    file_path (string): the file path to the file containing poses
  
  Returns: 
    A list of lists of floats. This represents a list of pose goals.
  """

  file = open(file_path, "r")
  file_text = file.read()
  file.close()

  file_lines = file_text.split("\n")
  result = []
  for line in file_lines:
    result_line = []
    line_split = line.split(",")
    for entry in line_split:
      entry.strip()
      result_line.append(float(entry))
    result.append(result_line)

  return result


def main(file_path):
  """
  Moves the end effector of the panda to the poses specified in the file given

  Args:
    file_path (string): the file path to the file containing poses
  """

  try:
    # Initializes the move group interface
    robot_interface = move_group_interface.MoveGroupInterface()

    # Parses the poses from the input file path
    pose_goals = parse_csv(file_path)

    # Goes to every pose in the list of poses
    for pose_goal in pose_goals:
      print("Moving to x = " + str(pose_goal[0]) + ", y = " +
            str(pose_goal[1]) + ", z = " + str(pose_goal[2]) + "...")
      robot_interface.go_to_pose(pose_goal[0], pose_goal[1], pose_goal[2])
      print("Waiting " + str(pose_goal[3]) + " seconds...")
      time.sleep(pose_goal[3])

  # Ends the program if we get one of these errors
  except rospy.ROSInterruptException:
    return
  except KeyboardInterrupt:
    return


if __name__ == '__main__':
  # Default file we are parsing poses from
  file_path = "/home/joseph/ws_moveit/src/jgioia_panda_projects/csv/ex1.csv"

  # Overrides default if file is specified in arguments
  if len(sys.argv) > 1:
    file_path = sys.argv[1]

  main(file_path)
