cmake_minimum_required(VERSION 3.5)
project(pose_utils)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_FLAGS "-O3 -Wall -g")

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(Armadillo REQUIRED)

add_library(pose_utils src/pose_utils.cpp)
target_include_directories(pose_utils
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${ARMADILLO_INCLUDE_DIRS}
)
target_link_libraries(pose_utils ${ARMADILLO_LIBRARIES})
ament_target_dependencies(pose_utils rclcpp)

install(TARGETS pose_utils
  EXPORT pose_utilsTargets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

ament_export_targets(pose_utilsTargets HAS_LIBRARY_TARGET)
ament_export_dependencies(rclcpp)
ament_package()
