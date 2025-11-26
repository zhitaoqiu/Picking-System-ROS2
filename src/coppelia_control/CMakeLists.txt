cmake_minimum_required(VERSION 3.5)
project(coppelia_control)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ---------------------------
# Dependencies
# ---------------------------
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_msgs REQUIRED)

# ---------------------------
# Library
# ---------------------------
add_library(coppelia_control_lib SHARED
  src/PlanningInterface.cpp
  src/ManipulationInterface.cpp
  src/VisionInterface.cpp
)

ament_target_dependencies(coppelia_control_lib
  rclcpp
  sensor_msgs
  geometry_msgs
  moveit_ros_planning_interface
  moveit_msgs
)

target_include_directories(coppelia_control_lib
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

install(
  TARGETS coppelia_control_lib
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY include/
  DESTINATION include/
)

# ---------------------------
# Main node executable
# ---------------------------
add_executable(main_node src/main.cpp)
target_link_libraries(main_node
  coppelia_control_lib
)

ament_target_dependencies(main_node
  rclcpp
  sensor_msgs
  geometry_msgs
  moveit_ros_planning_interface
  moveit_msgs
)

install(
  TARGETS main_node 
  DESTINATION lib/${PROJECT_NAME}
)

# ---------------------------
# Install config & launch
# ---------------------------
install(
  DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

install(
  DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

ament_package()
