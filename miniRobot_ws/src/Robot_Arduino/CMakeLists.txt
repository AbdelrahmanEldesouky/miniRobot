cmake_minimum_required(VERSION 3.0.2)
project(Robot_Arduino)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
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

add_executable(odom_basic src/Odom_basic.cpp)
target_link_libraries(odom_basic ${catkin_LIBRARIES})

add_executable(odom_translator src/Odom.cpp)
target_link_libraries(odom_translator ${catkin_LIBRARIES})

add_executable(imu_translator src/IMU.cpp)
target_link_libraries(imu_translator ${catkin_LIBRARIES})

add_executable(teleop_to_cmd src/teleop_to_cmdvel.cpp)
target_link_libraries(teleop_to_cmd ${catkin_LIBRARIES})


