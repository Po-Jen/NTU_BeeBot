cmake_minimum_required(VERSION 2.8.3)
project(mobile_safeguard_primitive)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  pcl_conversions
  pcl_ros
  roscpp
  rospy
  sensor_msgs
  std_msgs
  image_transport
  cv_bridge
  tf
  message_generation
  nodelet
  geometry_msgs
  gridmap_2d
#  riskrrt
  visualization_msgs
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)
find_package(PkgConfig REQUIRED)
pkg_check_modules(SBPL REQUIRED sbpl)

## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependencies might have been
##     pulled in transitively but can be declared for certainty nonetheless:
##     * add a build_depend tag for "message_generation"
##     * add a run_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
 add_message_files(
   FILES
   OccupancyGridArray.msg
   PoseTwistStamped.msg
 )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

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
   nav_msgs
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
# not needed for SBPL with CMake config
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
#  LIBRARIES riskrrt
  DEPENDS sbpl  
  CATKIN_DEPENDS message_runtime
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${SBPL_INCLUDE_DIRS}
  ${SBPL_LIBRARY_DIRS}
  ${OpenCV_INCLUDE_DIRS} 
  ${Boost_INCLUDE_DIRS}
  ${mobile_safeguard_primitive_INCLUDE_DIRS}
  ${SBPL_LIBRARY_DIRS}
#  ${riskrrt_INCLUDE_DIRS}
)

#include_directories(${PROJECT_SOURCE_DIR}/include/)
#set (HEADERS ${PROJECT_SOURCE_DIR}/include/people_filtering.h)

#add_library(${PROJECT_NAME} src/beebot_DStarLite_planner.cpp)

## Declare a cpp library
# add_library(mobile_safeguard_primitive
#   src/${PROJECT_NAME}/mobile_safeguard_primitive.cpp
# )

#add_library(ADStar_planner src/adstar_planner.cpp
#)


## Declare a cpp executable
add_executable(mobile_safeguard_primitive_node  src/mobile_safeguard_primitive.cpp)
add_executable(slammot_y_shaped_tube            src/slammot_y_shaped_tube.cpp)
#add_executable(preProcessing                    src/preProcessing.cpp)
#  I commented out people_filter because temporarily it is not possible to link to spencer_tracking_msgs/DetectedPersons.h
#add_executable(people_filter                    src/people_filter.cpp)
#add_executable(beebot_2d_planner_node           src/beebot_planner_2d.cpp)
#add_executable(path2traj_converter              src/path2traj_conv.cpp)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
# add_dependencies(mobile_safeguard_primitive_node mobile_safeguard_primitive_generate_messages_cpp)
#add_dependencies(path2traj_converter            ${catkin_EXPORTED_TARGETS})



## Specify libraries to link a library or executable target against
target_link_libraries(mobile_safeguard_primitive_node   ${catkin_LIBRARIES})
target_link_libraries(slammot_y_shaped_tube             ${catkin_LIBRARIES})
#target_link_libraries(preProcessing                     ${catkin_LIBRARIES} 
#                                                        ${PCL_LIBRARIES})
#target_link_libraries(people_filter                     ${catkin_LIBRARIES}
#                                                        ${PCL_LIBRARIES})
#target_link_libraries(beebot_2d_planner_node            ${PROJECT_NAME}
#                                                        ${catkin_LIBRARIES})
#target_link_libraries(path2traj_converter               ${catkin_LIBRARIES})
#target_link_libraries(${PROJECT_NAME}                   ${SBPL_LIBRARIES} 
#                                                        ${catkin_LIBRARIES})


#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS mobile_safeguard_primitive mobile_safeguard_primitive_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )
#install(TARGETS ${PROJECT_NAME}
#        DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#)
#install(TARGETS beebot_2d_planner_node
#        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#)


