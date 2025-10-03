## Move It2 Setup for Unitree Go2 with D1 arm.

1) Add the following packages to the src folder of your ROS2 workspace.
   
2)  Download unitree_sdk2 and install it globally.

3) Build the d1_sdk separately using cmake.. and make.

   
4) Use this following command in the terminal before using any script that involves the use of sdk:

export LD_LIBRARY_PATH=~/path/to/d1_sdk/lib:/usr/local/lib:$LD_LIBRARY_PATH
