cmake_minimum_required(VERSION 3.0.2)
project(tartan_vo_ros)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  dynamic_reconfigure
  image_transport
  message_runtime
  roscpp
  rospy
  sensor_msgs
  std_msgs
  tf
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES tartan_vo_ros
#  CATKIN_DEPENDS cv_bridge dynamic_reconfigure image_transport message_runtime roscpp rospy sensor_msgs std_msgs tf
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

#############
## Install ##
#############

catkin_install_python(PROGRAMS
  scripts/tartanvo_node.py
  scripts/Datasets/__init__.py 
  scripts/Datasets/tartanTrajFlowDataset.py
  scripts/Datasets/transformation.py
  scripts/Datasets/utils.py
  scripts/Network/__init__.py 
  scripts/Network/VOFlowNet.py
  scripts/Network/VONet.py
  scripts/Network/PWC/__init__.py
  scripts/Network/PWC/correlation.py
  scripts/Network/PWC/PWCNet.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)


install(DIRECTORY 
    models
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

