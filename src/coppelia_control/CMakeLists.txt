cmake_minimum_required(VERSION 3.8)
project(coppelia_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 1. 查找依赖包 (保留这些是为了让 ROS 知道环境依赖)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# =========================================================
#  C++ 编译区域 (已注释)
#  如果你现在只用 Python 脚本控制，这部分是不需要的。
#  注释掉可以避免 C++ 编译报错 (如找不到头文件、链接错误等)。
# =========================================================

# include_directories(include)

# add_library(manipulation_interface SHARED src/ManipulationInterface.cpp)
# ament_target_dependencies(manipulation_interface rclcpp moveit_msgs geometry_msgs)

# add_library(planning_interface SHARED src/PlanningInterface.cpp)
# ament_target_dependencies(planning_interface rclcpp moveit_msgs geometry_msgs)

# add_executable(franka_control_node src/franka_control_node.cpp)
# target_link_libraries(franka_control_node manipulation_interface planning_interface)
# ament_target_dependencies(franka_control_node rclcpp moveit_msgs geometry_msgs)

# install(TARGETS manipulation_interface planning_interface
#   ARCHIVE DESTINATION lib
#   LIBRARY DESTINATION lib
#   RUNTIME DESTINATION bin
# )
# install(TARGETS franka_control_node DESTINATION lib/${PROJECT_NAME})
# install(DIRECTORY include/ DESTINATION include)

# =========================================================
#  安装区域 (核心保留)
# =========================================================

# 2. 安装 Python 脚本 (你的大脑)
install(PROGRAMS
  scripts/grasp_manager.py
  DESTINATION lib/${PROJECT_NAME}
)

# 3. 安装 Launch 文件和 Config 文件 (你的身体配置)
# 注意：这里删除了 URDF 的安装，因为通常 URDF 不放在 control 包里，
# 即使放了，也可以通过 install(DIRECTORY ...) 一起装进去。
install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

# 4. 测试依赖 (保留)
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# 5. 导出依赖和结束
ament_package()