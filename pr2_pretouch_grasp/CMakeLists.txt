cmake_minimum_required(VERSION 2.8.3)
project(pr2_pretouch_grasp)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  moveit_msgs
  octomap_server
  pcl_ros
  pr2_pretouch_msgs
  roscpp
  rospy
  sensor_msgs
  std_msgs
  std_srvs
  tf
  visualization_msgs
)

## System dependencies are found with CMake's conventions
#find_package(PCL REQUIRED)

## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

#######################################
## Declare ROS messages and services ##
#######################################

## Generate messages in the 'msg' folder
# add_message_files(
#  FILES
# )

## Generate services in the 'srv' folder
#add_service_files(
#  FILES
#  ExecutePretouch.srv
#  findPretouch.srv
#  getProbabilisticPointCloud.srv
#  GraspPlanning.srv
#)

## Generate added messages and services with any dependencies listed here
#generate_messages(
#  DEPENDENCIES
#  moveit_msgs geometry_msgs  pr2_pretouch_msgs   sensor_msgs   std_msgs   visualization_msgs household_objects_database_msgs manipulation_msgs
#)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
 INCLUDE_DIRS include
 LIBRARIES pr2_pretouch_grasp tabletop_octomap
 CATKIN_DEPENDS moveit_msgs octomap_server pcl_ros pr2_pretouch_msgs roscpp rospy sensor_msgs std_msgs std_srvs tf visualization_msgs
 DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include)
include_directories(${catkin_INCLUDE_DIRS})

## Declare a cpp library
add_library(tabletop_octomap src/TabletopOctomapServer.cpp)

## Declare a cpp executable
add_executable(tabletop_octomap_server_node src/tabletop_octomap_server_node.cpp)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
#add_dependencies(pr2_pretouch_grasp_node pr2_pretouch_grasp_generate_messages_cpp)
#add_dependencies(src/TabletopOctomapServer.cpp ${PROJECT_NAME}_gencpp)

## Specify libraries to link a library or executable target against
target_link_libraries(tabletop_octomap ${catkin_LIBRARIES})
target_link_libraries(tabletop_octomap_server_node ${catkin_LIBRARIES} tabletop_octomap)

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_pr2_pretouch_grasp.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
