cmake_minimum_required(VERSION 3.8)
project(cs_hardware_interface)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(PkgConfig REQUIRED)

# 查找 ZMQ
pkg_check_modules(ZMQ REQUIRED libzmq)

add_library(cs_hardware_interface SHARED
  src/cs_system.cpp
)

target_include_directories(cs_hardware_interface PUBLIC include)

ament_target_dependencies(cs_hardware_interface
  rclcpp
  hardware_interface
  pluginlib
  rclcpp_lifecycle
)

# ZMQ 配置
target_include_directories(cs_hardware_interface PUBLIC ${ZMQ_INCLUDE_DIRS})
target_link_libraries(cs_hardware_interface ${ZMQ_LIBRARIES})

# 导出插件描述文件
pluginlib_export_plugin_description_file(hardware_interface cs_hardware_interface.xml)

# ================= 安装规则 (Install Rules) =================

# 1. 安装库文件 (so文件)
install(TARGETS cs_hardware_interface
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# 2. 【关键补充】安装 XML 描述文件
# 这一步如果不加，ROS 运行时会找不到插件！
install(FILES cs_hardware_interface.xml
  DESTINATION share/${PROJECT_NAME}
)

# 3. 安装头文件
install(DIRECTORY include/
  DESTINATION include
)

# 4. 安装 launch, config, urdf 文件夹
install(DIRECTORY launch config urdf
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
