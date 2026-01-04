cmake_minimum_required(VERSION 3.5)

project(gcopter)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -fPIC")

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(quadrotor_msgs REQUIRED)
find_package(plan_env REQUIRED)
find_package(cv_bridge REQUIRED)

find_package(Eigen3 REQUIRED)
find_package(ompl REQUIRED)

include_directories(
  ${PROJECT_SOURCE_DIR}/include
  ${OMPL_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  include
)

add_executable(global_planning src/global_planning.cpp)
target_link_libraries(global_planning ${OMPL_LIBRARIES})
ament_target_dependencies(global_planning
  rclcpp
  std_msgs
  geometry_msgs
  sensor_msgs
  nav_msgs
  visualization_msgs
  quadrotor_msgs
  plan_env
  cv_bridge
)

add_executable(traj_server_node src/traj_server.cpp)
ament_target_dependencies(traj_server_node
  rclcpp
  std_msgs
  quadrotor_msgs
  visualization_msgs
)

add_executable(traj_server_transport_node src/traj_server_transport.cpp)
ament_target_dependencies(traj_server_transport_node
  rclcpp
  std_msgs
  quadrotor_msgs
  visualization_msgs
)

install(TARGETS
  global_planning
  traj_server_node
  traj_server_transport_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

ament_package()
