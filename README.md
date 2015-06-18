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

## 2.2 use Beebot to run gmapping node
Launch the tf and laser on Beebot:

	roslaunch mobile_safeguard_primitive start_tf.launch

Run gmapping
	
	roslaunch mobile_safeguard_primitive gmapping.launch

Run Driver:

	roslaunch tobotdrivers TobotDriver.launch

Run Odometry:

	rosrun odom_base_pubisher odom_base_pubisher_node 

Use teleop to control robot fot building map:

	rosrun teleop_twist_keyboard teleop_twist_keyboard.py

Save the map if you're satisfied:

	rosrun map_server map_saver -f map
