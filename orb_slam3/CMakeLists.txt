cmake_minimum_required(VERSION 3.0.2)
project(orb_slam3)
set(ENV{ORBSLAM3_SOURCE_DIR} [$ENV{HOME}/dev/SLAM/ORB_SLAM3])

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED)

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -Wall  -O3 -march=native ")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall  -O3 -march=native")

# Check C++11 or C++0x support
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++14" COMPILER_SUPPORTS_CXX14)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX14)
   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14")
   add_definitions(-DCOMPILEDWITHC11)
   message(STATUS "Using flag -std=c++14.")
elseif(COMPILER_SUPPORTS_CXX0X)
   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
   add_definitions(-DCOMPILEDWITHC0X)
   message(STATUS "Using flag -std=c++0x.")
else()
   message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++14 support. Please use a different C++ compiler.")
endif()

LIST(APPEND CMAKE_MODULE_PATH ${ORBSLAM3_SOURCE_DIR}/../../../cmake_modules)

find_package(OpenCV)
   if(NOT OpenCV_FOUND)
      message(FATAL_ERROR "OpenCV > 4.4 not found.")
   endif()

find_package(Eigen3 3.4.0 REQUIRED)
find_package(Pangolin REQUIRED)


include_directories(
${ORBSLAM3_SOURCE_DIR}
${ORBSLAM3_SOURCE_DIR}/../../../
${ORBSLAM3_SOURCE_DIR}/../../../include
${ORBSLAM3_SOURCE_DIR}/../../../include/CameraModels
${Pangolin_INCLUDE_DIRS}
)

set(LIBS 
${OpenCV_LIBS} 
${EIGEN3_LIBS}
${Pangolin_LIBRARIES}
${ORBSLAM3_SOURCE_DIR}/../../../Thirdparty/DBoW2/lib/libDBoW2.so
${ORBSLAM3_SOURCE_DIR}/../../../Thirdparty/g2o/lib/libg2o.so
${ORBSLAM3_SOURCE_DIR}/../../../lib/libORB_SLAM3.so
-lboost_system
)


catkin_package()

# Node for monocular camera
add_executable(Mono src/ros_mono.cc)
target_link_libraries(Mono${LIBS})

# Node for stereo camera
add_executable(Stereo src/ros_stereo.cc)
target_link_libraries(Stereo ${LIBS})

# Node for RGB-D camera
rosbuild_add_executable(RGBD src/ros_rgbd.cc)
target_link_libraries(RGBD ${LIBS})

# Node for monocular camera with IMU
add_executable(Mono_Inertial src/ros_mono_inertial.cc)
target_link_libraries(Mono_Inertial ${LIBS})

# Node for stereo-inertial camera
rosbuild_add_executable(Stereo_Inertial src/ros_stereo_inertial.cc)
target_link_libraries(Stereo_Inertial ${LIBS})

include_directories()
