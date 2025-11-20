cmake_minimum_required(VERSION 3.5)
project(coppelia_control)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 依赖
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_msgs REQUIRED)

include_directories(
  include
)

# ================
# 1. 编译库
# ================
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

# 安装库
install(TARGETS coppelia_control_lib
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# 安装头文件
install(DIRECTORY include/
  DESTINATION include/
)

# ================
# 2. 编译 main 节点
# ================
add_executable(main_node src/main.cpp)

# 链接上库
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

# 安装可执行文件
install(TARGETS main_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
