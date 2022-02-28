cmake_minimum_required(VERSION 3.0.2)
project(Robot_filterdPose)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  nav_msgs
  tf
)

###################################
## catkin specific configuration ##
###################################

catkin_package()

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(filtered_pose src/filterd_odom.cpp)
target_link_libraries(filtered_pose ${catkin_LIBRARIES})
