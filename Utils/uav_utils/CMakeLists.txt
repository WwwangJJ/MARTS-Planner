cmake_minimum_required(VERSION 3.5)
project(uav_utils)

set(CMAKE_VERBOSE_MAKEFILE "false")
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -O3 -g")

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)

install(PROGRAMS
  scripts/odom_to_euler.py
  scripts/send_odom.py
  scripts/tf_assist.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_export_dependencies(rclcpp)
ament_package()
