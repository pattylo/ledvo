cmake_minimum_required(VERSION 3.0.2)
project(visual_odometry)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++17)
## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages

find_package(catkin COMPONENTS
  nodelet
  roscpp
  std_msgs
  roslaunch
  geometry_msgs
  mavros_msgs
  cv_bridge
  image_transport
  sensor_msgs
  message_generation
  genmsg
  visualization_msgs
  tf
)

find_package(OpenCV 4 REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Sophus REQUIRED)

###### GTSAM STUFF STARTS HERE

# Include GTSAM CMake tools
find_package(GTSAMCMakeTools)
#include(GtsamBuildTypes) # Load build type flags and default to Debug mode
#include(GtsamTesting)    # Easy functions for creating unit tests and scripts
#include(GtsamMatlabWrap) # Automatic MATLAB wrapper generation

# Ensure that local folder is searched before library folders
#include_directories(BEFORE "${PROJECT_SOURCE_DIR}")

###################################################################################
# Find GTSAM components
find_package(GTSAM REQUIRED) # Uses installed package
include_directories(${GTSAM_INCLUDE_DIR})

find_package(GTSAM_UNSTABLE REQUIRED) # Uses installed package
include_directories(${GTSAM_UNSTABLE_INCLUDE_DIR})

roslaunch_add_file_check(launch)


add_message_files(
  FILES
  alan_log.msg
)


generate_messages(DEPENDENCIES std_msgs sensor_msgs)

catkin_package(
  #  INCLUDE_DIRS include
  #  LIBRARIES offb
  CATKIN_DEPENDS 
  geometry_msgs 
  mavros_msgs 
  roscpp
  std_msgs
  message_generation
)

set(Sophus_LIBRARIES libSophus.so)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${Sophus_INCLUDE_DIRS}
)

add_library(alan
  src/led.cpp
  src/tools/RosTopicConfigs.cpp
  )

## Specify libraries to link a library or executable target against
target_link_libraries(
  alan
  ${catkin_LIBRARIES}
  ${OpenCV_INCLUDE_DIRS}
  ${Sophus_INCLUDE_DIRS}
  gtsam
  gtsam_unstable
)
add_dependencies(
  alan 
  visual_odometry_generate_messages_cpp
)

add_executable(lala
  src/lala.cpp
)

target_link_libraries(lala
  ${catkin_LIBRARIES}
  gtsam
  gtsam_unstable
)

add_dependencies(lala
  visual_odometry_generate_messages_cpp
  )







