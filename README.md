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

## 3.1 Cannot open /dev/ttyUSB0

If you can see your controller by 

	ls -l /dev/ttyUSB*
	
Then you probably don't have the access because your user is not in the dialout group, type:

	sudo adduser <username> dialout

## 3.2 Modify the node num of motors

You can set the node number bt Faulhaber's Motion Manager on Windows platform. 
If you could only use Linux, an alternative to detect the motors' corresponding node number is to use "minicom" to get the job done.Let's break down the debugging steps first on Windows and then on Linux.

### Windows

Once you have the Faulhaber Motion Manager installed and if you have many motor controllers at hand, make sure to check each controller one by one before moving on debugging for multiple motors at once. Have the motor connected to the controller and input appropriate voltage of 25V and 5A. Make sure that the logical output voltage is 5V. Check whether all the cables from the motor are correctly connected to the controller. Failure to do so may cause overheating to the motor.

Fire up Windows Hardware Manager, check which COM port is the controller connected. Launch Faulhaber Motion Manager and change to the corresponding COM port. If every setting is correct, the detected device should be shown in the center column of the software. At the input bar at the top left of Motion Manager, type 
    en
to enable the motor. There should be an "OK"status as feedback.
Now input at the same bar
    xv1000
the x represents the node number of the motor. If you don't know the number, right click the detected device status at the column and click "node info". A small window would pop up and you can look up the node number. the command sent to the controller should look somewhat like this: 1v1000. the last digits behind "v" indicates the rpm of the motor. Test different velocities with different mtor node number if you still have some trouble getting the motor to run, thpough such problem is rare, you should double check if the cables from the motor to the controller are correctly connected.

### Linux

By using minicom and by trial-and-error, we can get an educated guess about the node number. Although minicom is used to send directly th estrings to the connected devices, we can directly send out the same commands as we do in Fulhaber Motion Manager.
Make sure that the Baud rate setting is correct. For example, our controller has 9600 as the default Baud Rate. And then in minicom, send out the command
    en
    1v1000
    2v1000
and so on to find the correct node number for each servo motor.
minicom's usage is actually quite straight forward, press Ctrl+A, then release and press Z, a command hint table should pop up, press "E" and input the above command to run the motor.

Once you have the corresponding node number, and make sure that all the motors can run, let's proceed with modifying the node number in NTU Beebot.

### NTU Beebot

There are two aspects that need to keep an eye on. First is that the TobotDriver is running at the correct Baud Rate, and second, each wheel is assigned with the correct node number.
 
Go to the TobotDriver package directory, and modify the NTU_BeeBot/Tobotdriver/config/robot.yaml to let driver correctly talk to motors.

Then go under ~/TobotDriver/include/moyot_drivers/SerialPortControl_port.hpp, look for the line:

    public: 
    SerialPortControl(int Baud) ; motor(PORT), _verbose(false) {
    std::cout << "GOOOO" << std::endl ;
    _motor.OpenSerialPort::BAUD_115200, SerialPort::CHAR_SIZE_8, SerialPort::PARITY_NONE, SerialPort::STOP_BITS_1, SerialPort::FLOW_CONTROL_HARD ) ;
    }

change the "BAUD_112500" to "BAUD_9600" because 9600 is the default Baud Rate to which the motor can be synchronized.

Then, go to ~/TobotDriver/include/moyot_drivers/SerialPortControl_stream.hpp, look for the _pnode.param>std::string>("NodeleftWheel", _LWHEEL, "1");
_pnode.param>std::string>("NoderightWheel", _LWHEEL, "2");
_pnode.param>std::string>("Nodelifer", _LWHEEL, "3");

Change to the corresponding node number for each motor. A faster way to determine the node number is to detect the motors on Windows using the Faulhaber Motion Manager.

Don't forget to compile once all modifications are made.

**The original repo**

Most of the driver code is from https://github.com/MalcolmMielle/Tobot
