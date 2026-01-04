cmake_minimum_required(VERSION 3.5)
project(rviz_plugins)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rviz_common REQUIRED)
find_package(rviz_rendering REQUIRED)
find_package(pluginlib REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Widgets)

set(CMAKE_AUTOMOC ON)
add_definitions(-DQT_NO_KEYWORDS)

set(SOURCE_FILES
  src/pose_tool.cpp
  src/goal_tool.cpp
)

include_directories(include)

add_library(${PROJECT_NAME} SHARED ${SOURCE_FILES})

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  rviz_common
  rviz_rendering
  pluginlib
  tf2
  tf2_ros
  tf2_geometry_msgs
  geometry_msgs
)

target_link_libraries(${PROJECT_NAME} Qt5::Widgets)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(FILES
  plugin_description.xml
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY media/
  DESTINATION share/${PROJECT_NAME}/media
)

install(DIRECTORY icons/
  DESTINATION share/${PROJECT_NAME}/icons
)

ament_package()
