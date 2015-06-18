cmake_minimum_required(VERSION 2.8.3)
project(teleop_twist_keyboard)

find_package(catkin REQUIRED
geometry_msgs
message_generation
  rospy
  std_msgs)




## Generate services in the 'srv' folder
 add_service_files(
   FILES
   getobject.srv
   lifting.srv
   sentobject.srv
#   Service2.srv
 )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
 generate_messages(
   DEPENDENCIES
   geometry_msgs
   std_msgs

 )
 
 catkin_package(
 CATKIN_DEPENDS message_generation geometry_msgs roscpp rospy std_msgs)

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
catkin_install_python(PROGRAMS
   teleop_twist_keyboard.py
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

