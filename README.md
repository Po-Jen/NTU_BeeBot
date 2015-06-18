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


