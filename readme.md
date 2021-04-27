# jgioia_panda_projects

## Capabilities

This ROS Melodic package provides interfaces and launch files to use the Frank Emika Panda robot and the Intel Realsense R435i depth camera. It is made to expedite common functionalities for these tools. For the Panda, there is the MoveGroupInterface and a protobuf format to make moving and manipulating with the Panda easy. There are also several launch files that run models/simulations of the robot. The LinkPoseInterface provides a way to get the latest position of links from a gazebo simulation, in order to simulate object tracking. Finally, we also provide the MoveItObject class which allows us to simulate an obstacle and update its position. For the Intel Realsense R435i, we provide instructions on how to use it, and we provide a demo of real time object tracking and publishing with April tags. The DepthSensorInterface which was planned to do object tracking without April tags is incomplete for reasons discussed in section 6. Several demos exist for the functionalities mentioned. Finally, there is a skeleton testing script which has everything you need to start using the provided interfaces. Let me know if you’re having trouble with the package, and I’ll try to help.

## Table of contents

1. Installation
2. Movement
3. Robot Drivers
4. Object detection/avoidance
5. Difficulties encountered
6. Contact info

## Installation

### ROS packages from APT

ros-melodic-ros-core (std_msgs), ros-melodic-moveit (moveit_core, moveit_commander), ros-melodic-franka-ros (franka_description), ros-melodic-ros-comm (rospy, topic_tools), ros-melodic-viz (rviz), ros-melodic-moveit-resources (moveit_resources, moveit_resources_panda_moveit_config), ros-melodic-robot (xacro, robot_state_publisher, joint_state_publisher), ros-melodic-libfranka, ros-melodic-gazebo-ros-pkgs (gazebo_ros, gazebo_msgs), ros-melodic-geometry2 (tf2_geometry_msgs, tf2_ros), ros-melodic-ros-control (controller_manager), ros-melodic-ar-track-alvar (ar_track_alvar), ros-melodic-ar-track-alvar-msgs (ar_tack_alvar_msgs), ros-melodic-common-msgs (sensor_msgs)

* You may have many of these packages if you install the full version of ros
* Outside parentheses is the name to apt install
* Inside parentheses are the packages that are directly used

### ROS packages that need to be installed in your catkin workspace (if you don’t know how to do this, check the last section of installation)

[panda_moveit_config](https://github.com/ros-planning/panda_moveit_config/tree/melodic-devel), [panda_simulation](https://github.com/erdalpekel/panda_simulation)

### Non-ROS packages

[protobuf](https://askubuntu.com/questions/1072683/how-can-i-install-protoc-on-ubuntu-16-04), [numpy](https://pypi.org/project/numpy/)

### Installing my package in your catkin workspace

* Navigate to the source directory of your catkin workspace (likely ~/catkin_ws/src) (you will need to create a catkin workspace if you don’t have one). 
* git clone this package at that location.
* Go back to the main directory of the catkin workspace (ex. ~/catkin_ws).
* Use catkin_make or catkin build to build the package depending on what you have used before (it does not matter for this package, as long as it is consistent with what you’ve used).
* Make sure you have sourced the setup script (“source devel/setup.bash” if it is not in your .bashrc)
* Try launching a file

## Motion Planning

This package uses MoveIt for motion planning and execution. MoveIt allows us to use a variety of motion planning algorithms for our robot. In simulation, it uses a fake controller to emulate the movement of the robot. The controller configuration specifies what options MoveIt has for moving a joint in a joint group. The most common type of controller is the position based controller, but there are also velocity and effort based controllers. The real robot has controllers that work in a similar way to the fake controllers, and so MoveIt can use these too. We can request motion planning from MoveIt in 2 different ways. We can publish to MoveIt’s topics directly or we can use the MoveGroupCommander.

My package provides the MoveGroupInterface which has many convenience methods that call the MoveGroupCommander on the two joint groups in the Panda, which are the gripper and the rest of the robot. I will now briefly go over some of the methods in the MoveGroupInterface (for a full list check the page dedicated to motion planning). For gripping, we can close the gripper, open the gripper, or go to a certain width with the gripper.

```Python
moveGroupInterface = MoveGroupInterface()
moveGroupInterface.close_gripper() # Opens grippers
moveGroupInterface.open_gripper() # Closes grippers
```

moveGroupInterface.go_to_hand_joint_goal([0.01, 0.01]) # Each gripper is now 0.01m from closed
For moving the rest of the robot, we can go to a pose specified by x position, y position, z position, roll, pitch, and yaw. Any unspecified position dimension results in going to the current position in that dimension. Any unspecified orientation parameter results in going to the default orientation (gripper facing down and parallel to y axis).

```Python
moveGroupInterface.go_to_pose(x=0.4, y=0.5, z=0.3, roll=pi, pitch=0, yaw=pi/2)
```

We also have the capability to grab a moveItObject, which is a type, provided by this package, that will be talked about more in object detection/avoidance. This method makes the robot go above the object, descend to it, and then grasp it.

```Python
moveGroupInterface.grab_object(moveItObject)
```

My package also provides a protobuf message protocol for the moveGroupInterface. The message protocol allows you to specify a sequence of moveGroupInterface commands. The purpose of this is to allow you to easily run movement sequences, swap out movement sequences and send movement sequences to other users. However, I never found a good use for this, because it was not easier to use than using the moveGroupInterface directly through a python script, and as such, I did not update it when I added new features to the moveGroupInterface. If there’s interest, I can update it with the latest features.

## Robot Drivers

There are three ways to run the robot. You can use RViz, which is a program that does not attempt to simulate physics, but has many visualization features. You can Gazebo, which attempts to simulate physics. Or you can run the program on the actual robot. RViz can be used in addition to the last two. Since I have not been able to test on the real robot, I will not cover how to use the real robot.

RViz is, as the name implies, a great robot visualizer. It allows us to do things like view the robot, view the image from a camera, view the depth cloud from a depth camera, and view objects known to the motion planning algorithm. We have several launch files that are exclusively for RViz. `panda_table.launch` launches the robot attached to the table. As mentioned previously, no physics are simulated for this. However you can still send commands to the robot launched here just like the simulated robot. `panda_table_camera.launch` does the same thing as panda_table.launch except it attaches the intel d435i camera and shows the depth cloud detected by the camera. The camera must be plugged in for this launch file to work properly. `demo_april_tag.launch` launches RViz in the same way as panda_table_camera.launch, but it has additional features we will discuss in the object detection section.

Gazebo is usually good at physics simulation, however it can be inconsistent and can cause unexpected behavior. When we launch a gazebo simulation, we also launch RViz so that we have additional visualization options. One thing to note is that just because an object is in Gazebo does not mean that it is known to the planning algorithm. Only the objects shown in the planning scene in RViz are known. We have 2 launch files for gazebo. `simulation.launch` is the basic simulation of the panda. It does not contain any additional objects. `demo_gazebo_manipulation.launch` is like simulation.launch with additional boxes to showcase object manipulation. You can run `demo_gazebo_manipulation.py` after you’ve launched `demo_gazebo_manipulation.launch` to watch the demo. Here’s a video of the demo:

## Object detection/avoidance

For my package’s purposes, there are 2 reasons we would want to detect an object. One reason would be to grab it. The other reason is to avoid it. For the second of these, we need to add the object to the planning algorithms planning scene (set of known objects). For the first, we do not want the object in the planning scene so that we can plan a path that collides with the object (ex. grabbing the object). The `MoveItObject` class handles both of these cases. To use the `MoveItObject`, create a `MoveItObject` specifying the type of object you want to create. I provide several examples built in, but it should be easy to add additional. By default, this object is added to the motion planner to avoid collisions, but by specifying `collision=False` in the constructor, you can use it for grabbing. You could also turn off collisions by calling the `delete()` method which removes the object from the planning scene, and then turn them back on with the `make()` method which adds the object to the planning scene. You can get the object’s current position with the `pose` field. You can move the object with `set_pose(pose)`, but be aware that this adds the object to the planning scene (if you just want to change the pose use the `pose` field). The reason for separate ways to do this is because to move an object in the planning scene we need to delete it and re-add it. The `MoveItObject` also stores a grasp pose and width which allow it to be grabbed by the MoveGroupInterface’s grabObject method. So far I have described how you would use a `MoveItObject` that you specify the position of, but what if you want to use a `MoveItObject` to track a real/simulated object.

The `GazeboObjectImporter` tracks the position of a link in Gazebo and updates the position of the `MoveItObject` provided. It takes, in its constructor, the `MoveItObject` to update and the link name. The `ArObjectImporter` does a similar thing. However, instead of tracking a link in gazebo, it uses the output of the ar_track_alvar package which tracks april tags using a camera. The `ArObjectImporter` is set up to translate the position published in the camera’s frame to the position in the robot’s frame. Both of these add the object to the planning scene whenever they receive an updated pose. However, you turn update off and on with the provided methods.

I will now briefly cover the april tag demo. demo_april_tag takes input from a camera to search for objects that have april tags on them and then attempts to grab them. To run this, connect the D435i camera, launch `demo_april_tag.launch`, and finally run `demo_april_tag.py`. Here’s a video of it working (Note that since we haven’t been able to test on the robot, I am moving the camera with my arm to simulate the robot moving):

## Difficulties encountered

I encountered a significant amount of problems when creating this package. You might also encounter these, so I am describing them here.

*Panda model:* I have had several difficulties with the robot’s model. There are many different models in different packages and different versions of packages, which have different collision models. The two biggest problems have been extremely conservative collision models and collision models that do not include the gripper. To check your model, in rviz check the show collision model box under robot model (also uncheck show visual model). You should be able to tell if there is a problem with your model.

*Conda with ROS:* Please do not try to use conda with ROS. I tried to and it broke my ROS installation, so that rospy no longer worked. This was difficult to fix.

*ROS/Python version support:* You may want to use Python 3 or a different version of ROS. However, there are many compatibility issues. For ROS Melodic, python 3 is not available, without significant difficulties. For later ROS versions, including ROS 2, Python 3 is supported, however many necessary packages are not. It looks like most packages are being migrated and ROS 2 may be reasonable to switch to by the end of the year.

*Pick and place in Gazebo:* There were several problems with pick and place in gazebo. The first is that the grippers would not work. To solve this, I had to remove the mimic tag from the srdf file of the robot, as Gazebo does not support mimic joints. Once the grippers work, there are also picking up an object in gazebo. Friction does not seem to work like it should and I have not been able to demonstrate that you can pick up an object with the grippers in Gazebo. I also have not found any other demo that can, and the simulation package I based my simulations on used a robot state publisher to effectively link a box to the gripper, mimicking picking it up.

*Object tracking:* You might notice that my package’s object tracking is limited to april tags and does not utilize the depth sensor. Originally we wanted to use ros_object_analytics. However, after spending a significant amount of time trying to get the package to work we could not. It depends on deprecated packages that depend on other packages. Further, none of the package worked the first time I installed them. There is a new version of ros_object_analytics for ROS 2, but as mentioned earlier we cannot switch to ROS 2 yet. There seems to be a lack of good options for object tracking in ROS Melodic, but it is possible that we just missed a good one.

*MoveItObject updating:* As mentioned before, MoveItObjects have to remove themselves from the planning scene and then re add themselves to change their pose. This can cause flickering of the objects when they are updating frequently (such as when you have multiple objects being tracked).

*MoveItObject make:* I had a lot of issues adding objects to a
planning scene that seemed to be very inconsistent. It ended up
resulting from the fact that the PlanningSceneInterface constructor
is asynchronous. To fix this, you should add `synchronous=True` to
the constructor of PlanningSceneInterface wherever you create an
instance.

## Contact info

Lastly, feel free to contact me at jeg4@rice.edu, if something is not working, as I might have dealt with the issue before.
