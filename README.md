## Move It2 Setup for Unitree Go2 with D1 arm.

1) Add the following packages to the src folder of your ROS2 workspace.
   
2)  Download unitree_sdk2 and install it globally.

3) Build the d1_sdk separately using cmake.. and make.

   
4) Use this following command in the terminal before using any script that involves the use of sdk:

**export LD_LIBRARY_PATH=~/path/to/d1_sdk/lib:/usr/local/lib:$LD_LIBRARY_PATH**

# INFO


To get the simulation and code running use the following commands:

1) **ros2 launch abb_arm gazebo.launch.py **-> This will launch the gazebo simulation and rviz window that can be used for MoveIt Motion Planning.
2) **ros2 run my_arm_control move_arm** -> This will interface the physical arm using the Ethernet connection.
3) **ros2 run move_program move_program **-> This will run a CPP script to move the end-effector to a particular position and orientation.
