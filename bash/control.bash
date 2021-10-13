cd panda_ws/devel
source setup.zsh
roslaunch franka_control franka_control.launch robot_ip:=192.168.1.5 load_gripper:=true
cd ~
