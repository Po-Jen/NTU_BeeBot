# NTU_BeeBot
Low-level control tools for using Beebot

# 1. Installation

## 1.1 ROS Groovy or Hydro + Ubuntu 12.04
Install system dependencies:

	sudo apt-get install libserial-dev

In your ROS package path, clone the repository: 

	git clone https://github.com/Po-Jen/NTU_BeeBot.git

Compile the package by typing:

	catkin_make

# 2. Usage

## 2.1 Test if the drvier works by teleop	
Launch the driver to control the robot by ROS topic:

	roslaunch tobotdrivers TobotDriver.launch
	
Execute teleop for testing if the driver works by:

	rosrun teleop_twist_keyboard teleop_twist_keyboard.py

## 2.2 2D SLAM by gmapping
Launch the tf and laser on Beebot:

	roslaunch mobile_safeguard_primitive start_tf.launch

Run gmapping
	
	roslaunch mobile_safeguard_primitive gmapping.launch

Fires up the driver:

	roslaunch tobotdrivers TobotDriver.launch

Run Odometry:

	rosrun odom_base_pubisher odom_base_pubisher_node 

Use teleop to control robot fot building map:

	rosrun teleop_twist_keyboard teleop_twist_keyboard.py

Save the map if you're satisfied:

	rosrun map_server map_saver -f map


## 2.3 2D Navigation
Launch the tf and laser:

	roslaunch mobile_safeguard_primitive start_tf.launch

Start move_base and amcl:

	roslaunch mobile_safeguard_primitive start_navigation.launch

Fires up the driver:

	roslaunch tobotdrivers TobotDriver.launch

Send the goal via Rviz:

	rosrun rviz rviz

## 2.4 Run RGBDSLAMv2 (only on Hydro)
For more details of this package, visit http://felixendres.github.io/rgbdslam_v2/ .

Install rgbdslam_v2:

	git clone https://github.com/felixendres/rgbdslam_v2.git
	sudo apt-get install ros-hydro-libg2o
	catkin_make

Run rgbdslam_v2 with Xtion(with GUI):

	roslaunch mobile_safeguard_primitive xtion+rgbdslam.launch

# 3. Troubleshoot / FAQ

**Cannot open /dev/ttyUSB0**

If you can see your controller by 

	ls -l /dev/ttyUSB*
	
Then you probably don't have the access because your user is not in the dialout group, type:

	sudo adduser <username> dialout

**Modify the node num of motors**

You can set the node number bt Faulhaber's Motion Manager. 
After setting the number, you should modify the NTU_BeeBot/Tobotdriver/config/robot.yaml to let driver correctly talk to motors.

**The original repo**

Most of the driver code is from https://github.com/MalcolmMielle/Tobot
