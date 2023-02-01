cmake_minimum_required(VERSION 3.0.2)
project(orb_slam3)
set(ENV{ORBSLAM3_SOURCE_DIR} $ENV{HOME}/dev/SLAM/ORB_SLAM3)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  cv_bridge
  image_transport
)

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

LIST(APPEND CMAKE_MODULE_PATH $ENV{ORBSLAM3_SOURCE_DIR}/cmake_modules)

find_package(OpenCV 4 QUIET )
   if(NOT OpenCV_FOUND)
      message(FATAL_ERROR "OpenCV > 4.4 not found.")
   endif()

find_package(Eigen3 3.4.0 REQUIRED)
find_package(Pangolin REQUIRED)

catkin_package (
  CATKIN_DEPENDS roscpp rospy
  LIBRARIES {PROJECT_NAME} libDBoW2 libg2o
)

include_directories(
$ENV{ORBSLAM3_SOURCE_DIR}
$ENV{ORBSLAM3_SOURCE_DIR}/include
$ENV{ORBSLAM3_SOURCE_DIR}/include/CameraModels
$ENV{ORBSLAM3_SOURCE_DIR}/Thirdparty/Sophus
${EIGEN3_INCLUDE_DIR}
${Pangolin_INCLUDE_DIRS}
${catkin_INCLUDE_DIRS}
/usr/include/opencv4/
)

add_library(${PROJECT_NAME} SHARED
$ENV{ORBSLAM3_SOURCE_DIR}/src/System.cc
$ENV{ORBSLAM3_SOURCE_DIR}/src/Tracking.cc
$ENV{ORBSLAM3_SOURCE_DIR}/src/LocalMapping.cc
$ENV{ORBSLAM3_SOURCE_DIR}/src/LoopClosing.cc
$ENV{ORBSLAM3_SOURCE_DIR}/src/ORBextractor.cc
$ENV{ORBSLAM3_SOURCE_DIR}/src/ORBmatcher.cc
$ENV{ORBSLAM3_SOURCE_DIR}/src/FrameDrawer.cc
$ENV{ORBSLAM3_SOURCE_DIR}/src/Converter.cc
$ENV{ORBSLAM3_SOURCE_DIR}/src/MapPoint.cc
$ENV{ORBSLAM3_SOURCE_DIR}/src/KeyFrame.cc
$ENV{ORBSLAM3_SOURCE_DIR}/src/Atlas.cc
$ENV{ORBSLAM3_SOURCE_DIR}/src/Map.cc
$ENV{ORBSLAM3_SOURCE_DIR}/src/MapDrawer.cc
$ENV{ORBSLAM3_SOURCE_DIR}/src/Optimizer.cc
$ENV{ORBSLAM3_SOURCE_DIR}/src/Frame.cc
$ENV{ORBSLAM3_SOURCE_DIR}/src/KeyFrameDatabase.cc
$ENV{ORBSLAM3_SOURCE_DIR}/src/Sim3Solver.cc
$ENV{ORBSLAM3_SOURCE_DIR}/src/Viewer.cc
$ENV{ORBSLAM3_SOURCE_DIR}/src/ImuTypes.cc
$ENV{ORBSLAM3_SOURCE_DIR}/src/G2oTypes.cc
$ENV{ORBSLAM3_SOURCE_DIR}/src/CameraModels/Pinhole.cpp
$ENV{ORBSLAM3_SOURCE_DIR}/src/CameraModels/KannalaBrandt8.cpp
$ENV{ORBSLAM3_SOURCE_DIR}/src/OptimizableTypes.cpp
$ENV{ORBSLAM3_SOURCE_DIR}/src/MLPnPsolver.cpp
$ENV{ORBSLAM3_SOURCE_DIR}/src/GeometricTools.cc
$ENV{ORBSLAM3_SOURCE_DIR}/src/TwoViewReconstruction.cc
$ENV{ORBSLAM3_SOURCE_DIR}/src/Config.cc
$ENV{ORBSLAM3_SOURCE_DIR}/src/Settings.cc
$ENV{ORBSLAM3_SOURCE_DIR}/include/System.h
$ENV{ORBSLAM3_SOURCE_DIR}/include/Tracking.h
$ENV{ORBSLAM3_SOURCE_DIR}/include/LocalMapping.h
$ENV{ORBSLAM3_SOURCE_DIR}/include/LoopClosing.h
$ENV{ORBSLAM3_SOURCE_DIR}/include/ORBextractor.h
$ENV{ORBSLAM3_SOURCE_DIR}/include/ORBmatcher.h
$ENV{ORBSLAM3_SOURCE_DIR}/include/FrameDrawer.h
$ENV{ORBSLAM3_SOURCE_DIR}/include/Converter.h
$ENV{ORBSLAM3_SOURCE_DIR}/include/MapPoint.h
$ENV{ORBSLAM3_SOURCE_DIR}/include/KeyFrame.h
$ENV{ORBSLAM3_SOURCE_DIR}/include/Atlas.h
$ENV{ORBSLAM3_SOURCE_DIR}/include/Map.h
$ENV{ORBSLAM3_SOURCE_DIR}/include/MapDrawer.h
$ENV{ORBSLAM3_SOURCE_DIR}/include/Optimizer.h
$ENV{ORBSLAM3_SOURCE_DIR}/include/Frame.h
$ENV{ORBSLAM3_SOURCE_DIR}/include/KeyFrameDatabase.h
$ENV{ORBSLAM3_SOURCE_DIR}/include/Sim3Solver.h
$ENV{ORBSLAM3_SOURCE_DIR}/include/Viewer.h
$ENV{ORBSLAM3_SOURCE_DIR}/include/ImuTypes.h
$ENV{ORBSLAM3_SOURCE_DIR}/include/G2oTypes.h
$ENV{ORBSLAM3_SOURCE_DIR}/include/CameraModels/GeometricCamera.h
$ENV{ORBSLAM3_SOURCE_DIR}/include/CameraModels/Pinhole.h
$ENV{ORBSLAM3_SOURCE_DIR}/include/CameraModels/KannalaBrandt8.h
$ENV{ORBSLAM3_SOURCE_DIR}/include/OptimizableTypes.h
$ENV{ORBSLAM3_SOURCE_DIR}/include/MLPnPsolver.h
$ENV{ORBSLAM3_SOURCE_DIR}/include/GeometricTools.h
$ENV{ORBSLAM3_SOURCE_DIR}/include/TwoViewReconstruction.h
$ENV{ORBSLAM3_SOURCE_DIR}/include/SerializationUtils.h
$ENV{ORBSLAM3_SOURCE_DIR}/include/Config.h
$ENV{ORBSLAM3_SOURCE_DIR}/include/Settings.h)

set(ROS_ORBSLAM_LIBS 
${catkin_LIBRARIES}
${OpenCV_LIBS} 
${EIGEN3_LIBS}
${Pangolin_LIBRARIES}
$ENV{ORBSLAM3_SOURCE_DIR}/Thirdparty/DBoW2/lib/libDBoW2.so
$ENV{ORBSLAM3_SOURCE_DIR}/Thirdparty/g2o/lib/libg2o.so
$ENV{ORBSLAM3_SOURCE_DIR}/lib/libORB_SLAM3.so
-lboost_system
)

# add_subdirectory($ENV{ORBSLAM3_SOURCE_DIR}/Thirdparty/g2o)
target_link_libraries(${PROJECT_NAME}
${ROS_ORBSLAM_LIBS}
)

# Node for monocular camera
add_executable(Mono src/ros_mono.cc)
target_link_libraries(Mono ${ROS_ORBSLAM_LIBS})

# Node for stereo camera
add_executable(Stereo src/ros_stereo.cc)
target_link_libraries(Stereo ${ROS_ORBSLAM_LIBS})

# Node for RGB-D camera
add_executable(RGBD src/ros_rgbd.cc)
target_link_libraries(RGBD ${ROS_ORBSLAM_LIBS})

# Node for monocular camera with IMU
add_executable(Mono_Inertial src/ros_mono_inertial.cc)
target_link_libraries(Mono_Inertial ${ROS_ORBSLAM_LIBS})

# Node for stereo-inertial camera
add_executable(Stereo_Inertial src/ros_stereo_inertial.cc)
target_link_libraries(Stereo_Inertial ${ROS_ORBSLAM_LIBS})

install(DIRECTORY config launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})


# include_directories()
