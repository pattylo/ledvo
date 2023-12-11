cmake_minimum_required(VERSION 3.16)
project(vdo)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++17)
## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages

# set(LIBTORCH_FOLDER_NAME "libtorch")
# set(CMAKE_PREFIX_PATH "${CMAKE_CURRENT_SOURCE_DIR}/${LIBTORCH_FOLDER_NAME}" ${CMAKE_PREFIX_PATH})

set(Torch_DIR "${CMAKE_CURRENT_SOURCE_DIR}/libtorch/share/cmake/Torch/")

find_package(catkin REQUIRED COMPONENTS
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

# The Usuals
find_package(OpenCV 4 REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Sophus REQUIRED)


# Torch
find_package(Torch REQUIRED)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${TORCH_CXX_FLAGS}")

# GTSAM
find_package(GTSAMCMakeTools) # Include GTSAM CMake tools
find_package(GTSAM REQUIRED) # Find GTSAM components
include_directories(${GTSAM_INCLUDE_DIR})

# find_package(GTSAM_UNSTABLE REQUIRED) # Uses installed package
# include_directories(${GTSAM_UNSTABLE_INCLUDE_DIR})

roslaunch_add_file_check(launch)

add_message_files(
  FILES
  ledvo_log.msg
)

generate_messages(DEPENDENCIES std_msgs sensor_msgs)

catkin_package(
  INCLUDE_DIRS include

  CATKIN_DEPENDS 
  geometry_msgs 
  mavros_msgs 
  roscpp
  std_msgs
  message_generation
)

set(Sophus_LIBRARIES libSophus.so)


###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${Sophus_INCLUDE_DIRS}
)

############################################################################
## Declare a C++ ledvo nodelet library

add_library(ledvo
  src/ledvo/ledvo.cpp
  src/ledvo/config.cpp
  src/ledvo/landmarks.cpp
  src/ledvo/dynamics.cpp
  src/ledvo/fgo.cpp
)

# Specify libraries to link a library or executable target against
target_link_libraries(ledvo
  ${catkin_LIBRARIES}
  ${OpenCV_INCLUDE_DIRS}
  ${Sophus_INCLUDE_DIRS}
  ${TORCH_LIBRARIES}
  gtsam
  # gtsam_unstable
)
add_dependencies(ledvo
  vdo_generate_messages_cpp
)

############################################################################
## Declare C++ executables

add_executable(lala
  src/lala/lala.cpp
)

target_link_libraries(lala
  ${catkin_LIBRARIES}
  gtsam
  # gtsam_unstable
  ${TORCH_LIBRARIES}
)

add_dependencies(lala
  vdo_generate_messages_cpp
)

############################################################################

add_executable(datalog
  src/datalog/datalog.cpp
)

target_link_libraries(datalog
  ${catkin_LIBRARIES}
)
