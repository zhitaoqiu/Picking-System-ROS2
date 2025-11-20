cmake_minimum_required(VERSION 3.5)
project(coppelia_control)

# 默认使用 C++17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 找依赖
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_msgs REQUIRED)

# 包含 include 目录
include_directories(
  include
)

# 你的 cpp 实现文件（后面加即可）
set(SOURCES
  src/planning_interface.cpp
  src/manipulation_interface.cpp
  src/vision_interface.cpp
)

# 编译库，供节点或其他包调用
add_library(${PROJECT_NAME} SHARED
  ${SOURCES}
)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  sensor_msgs
  geometry_msgs
  moveit_ros_planning_interface
  moveit_msgs
)

# 安装头文件
install(DIRECTORY include/
  DESTINATION include/
)

# 安装库
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_package()
