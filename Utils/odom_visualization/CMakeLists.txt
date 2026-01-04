cmake_minimum_required(VERSION 3.5)
project(odom_visualization)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_FLAGS "-O3 -Wall -g")

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(quadrotor_msgs REQUIRED)
find_package(pose_utils REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(Armadillo REQUIRED)

include_directories(
  include
  ${ARMADILLO_INCLUDE_DIRS}
)

add_executable(odom_visualization src/odom_visualization.cpp)

target_link_libraries(odom_visualization ${ARMADILLO_LIBRARIES})
ament_target_dependencies(odom_visualization
  rclcpp
  sensor_msgs
  nav_msgs
  visualization_msgs
  quadrotor_msgs
  pose_utils
  tf2
  tf2_ros
  tf2_geometry_msgs
)

install(TARGETS odom_visualization
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
