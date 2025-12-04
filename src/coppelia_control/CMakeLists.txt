cmake_minimum_required(VERSION 3.8)
project(coppelia_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 查找依赖包
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)

# 包含目录
include_directories(
  include
)

# 依赖项列表
set(THIS_PACKAGE_INCLUDE_DEPENDS
  rclcpp
  moveit_core
  moveit_ros_planning
  moveit_ros_planning_interface
  moveit_msgs
  geometry_msgs
  sensor_msgs
  trajectory_msgs
)

# 创建ManipulationInterface库
add_library(manipulation_interface SHARED
  src/ManipulationInterface.cpp
)

ament_target_dependencies(manipulation_interface
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

# 创建PlanningInterface库
add_library(planning_interface SHARED
  src/PlanningInterface.cpp
)

ament_target_dependencies(planning_interface
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

# 创建VisionInterface库（如果需要）
add_library(vision_interface SHARED
  src/VisionInterface.cpp
)

ament_target_dependencies(vision_interface
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

# 创建控制节点可执行文件
add_executable(franka_control_node
  src/franka_control_node.cpp
)

target_link_libraries(franka_control_node
  manipulation_interface
  planning_interface
)

ament_target_dependencies(franka_control_node
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

# 安装目标
install(TARGETS
  manipulation_interface
  planning_interface
  vision_interface
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS
  franka_control_node
  DESTINATION lib/${PROJECT_NAME}
)

# 安装头文件
install(DIRECTORY include/
  DESTINATION include
)

# 安装启动文件和配置文件
install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)
install(
  FILES urdf/Franka.urdf
  DESTINATION share/${PROJECT_NAME}/urdf
)
install(
  DIRECTORY meshes/
  DESTINATION share/${PROJECT_NAME}/meshes
)
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_export_libraries(manipulation_interface planning_interface vision_interface)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})

ament_package()