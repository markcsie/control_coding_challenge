cmake_minimum_required(VERSION 2.8.3)
project(control_coding_challenge)

## Add support for C++11, supported in ROS Kinetic and newer
add_definitions(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  genmsg
)

## System dependencies are found with CMake's conventions
find_package(Eigen3 REQUIRED)

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  NoiseMatrix.msg
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs  # Or other packages containing msgs
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
add_executable(${PROJECT_NAME}_node
    src/control_coding_challenge_node.cpp
    src/sensor_fusion.cpp

    include/sensor_fusion.h # workaround for qt?
)

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
)
