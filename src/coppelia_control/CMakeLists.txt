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
find_package(xacro REQUIRED)

include_directories(
  include
)

# ================
# 1. 生成 robot_description (panda.urdf)
# ================
set(URDF_INPUT urdf/panda.urdf)
set(URDF_OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/panda.urdf)

configure_file(${URDF_INPUT} ${URDF_OUTPUT} COPYONLY)

# ================
# 2. 生成 robot_description_semantic (panda.srdf)
#    如果你未来准备用 xacro，就改成 xacro 处理即可
# ================
set(SRDF_INPUT srdf/panda.srdf)
set(SRDF_OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/panda.srdf)

configure_file(${SRDF_INPUT} ${SRDF_OUTPUT} COPYONLY)

# 安装 URDF/SRDF 到 share/coppelia_control/
install(
  FILES
    ${URDF_INPUT}
    ${SRDF_INPUT}
  DESTINATION share/${PROJECT_NAME}/
)

# ================
# 3. 编译库
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

install(TARGETS coppelia_control_lib
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include/
)

# ================
# 4. main_node 可执行文件
# ================
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

install(TARGETS main_node
  DESTINATION lib/${PROJECT_NAME}
)

# ================
# 5. 安装 launch
# ================
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

ament_package()

