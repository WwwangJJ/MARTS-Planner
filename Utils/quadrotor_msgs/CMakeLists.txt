#-cmake_minimum_required(VERSION 2.4.6)
#-include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

#-rosbuild_init()

#set the default path for built executables to the "bin" directory
#-set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
#-set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

#common commands for building c++ executables and libraries
#-\rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})

#-rosbuild_add_library(encode_msgs src/encode_msgs.cpp)
#-rosbuild_add_link_flags(encode_msgs -Wl,--as-needed)

#list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)
#find_package(Eigen3 REQUIRED)

#include_directories(${EIGEN3_INCLUDE_DIR})
#-rosbuild_add_library(decode_msgs src/decode_msgs.cpp)
#-rosbuild_add_link_flags(decode_msgs -Wl,--as-needed)

#------------------------------------------------------------------
cmake_minimum_required(VERSION 3.5)
project(quadrotor_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

set(msg_files
  "msg/AuxCommand.msg"
  "msg/Corrections.msg"
  "msg/Gains.msg"
  "msg/Rpms.msg"
  "msg/OutputData.msg"
  "msg/PositionCommand.msg"
  "msg/PolyTraj.msg"
  "msg/PositionCommandNew.msg"
  "msg/PositionCommandLoad.msg"
  "msg/PPROutputData.msg"
  "msg/SimState.msg"
  "msg/IndiState.msg"
  "msg/Serial.msg"
  "msg/SO3Command.msg"
  "msg/StatusData.msg"
  "msg/TRPYCommand.msg"
  "msg/Odometry.msg"
  "msg/PolynomialTrajectory.msg"
  "msg/LQRTrajectory.msg"
  "msg/geDebug.msg"
  "msg/Px4ctrlDebug.msg"
  "msg/TakeoffLand.msg"
  "msg/trigger.msg"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  DEPENDENCIES geometry_msgs nav_msgs
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
