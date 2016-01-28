cmake_minimum_required(VERSION 2.8.3)
project(beebot_rrt_planner)
ADD_DEFINITIONS(-std=c++0x) # C++ 11 is required

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_core
  nav_msgs
  pluginlib
  costmap_2d
  navfn
	global_planner
  roscpp
  rospy
  std_msgs
  tf
  visualization_msgs
)

find_package(cmake_modules REQUIRED)



# add qt4 to the project
FIND_PACKAGE(Eigen REQUIRED)
FIND_PACKAGE(Boost REQUIRED)



catkin_package(

  CATKIN_DEPENDS
    geometry_msgs
    nav_core
    nav_msgs
    roscpp
    rospy
    std_msgs
    tf
    visualization_msgs
    costmap_2d
    pluginlib
    navfn
		global_planner

  LIBRARIES
    beebot_smp_planner


)


FIND_PACKAGE( OpenMP REQUIRED)
if(OPENMP_FOUND)
  message("OPENMP FOUND")
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
  set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
endif()


###########
## Build ##
###########
include_directories(${Eigen_INCLUDE_DIRS})

include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}/include/
  ${CMAKE_CURRENT_SOURCE_DIR}/src/
  ${CMAKE_CURRENT_SOURCE_DIR}
  ${catkin_INCLUDE_DIRS}
)


# RRT PLANNER

## Declare KdTree lib
add_library(BEEBOT_RRT_PLANNER_KDTREE_LIB ${CMAKE_CURRENT_SOURCE_DIR}/include/smp/external_libraries/kdtree/kdtree.c)


# Setting SOURCES variables
set(SOURCES_RRT src/beebot_smp_planner.cpp include/beebot_rrt_planner/beebot_smp_planner.h)

set(SOURCES
    src/costmap_layers_dyn_rec_handler.cpp 
    include/${PROJECT_NAME}/costmap_layers_dyn_rec_handler.h 
    src/thetastar_leading_rrt.cpp 
    include/${PROJECT_NAME}/thetastar_leading_rrt.h 
    include/${PROJECT_NAME}/costmap_model.cpp 
    include/${PROJECT_NAME}/trajectory.cpp 
    include/${PROJECT_NAME}/grid.cpp 
    include/${PROJECT_NAME}/cell.cpp

)



# Plugin
add_library(beebot_smp_planner  ${SOURCES_RRT} ${SOURCES})

target_link_libraries(beebot_smp_planner ${catkin_LIBRARIES} ${BOOST_LIBRARIES} BEEBOT_RRT_PLANNER_KDTREE_LIB)

add_dependencies(beebot_smp_planner beebot_rrt_planner_gencfg ${catkin_EXPORTED_TARGETS})

add_dependencies(beebot_smp_planner nav_msgs_gencpp)


install(TARGETS beebot_smp_planner
       ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
       LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}

       )

install(FILES beebot_smp_planner_plugin.xml
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)


install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  PATTERN ".svn" EXCLUDE
)
