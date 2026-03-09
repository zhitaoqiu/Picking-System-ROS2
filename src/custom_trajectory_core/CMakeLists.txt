cmake_minimum_required(VERSION 3.8)
project(custom_trajectory_core)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

# 1. 编译动态链接库 (Shared Library)
add_library(${PROJECT_NAME} SHARED
  src/path_smoother.cpp         
  src/trapezoidal_planner.cpp   
)

# ================= 新增核心逻辑开始 =================

# 2. 指定头文件搜索路径（解决找不到 hpp 的报错）
# PUBLIC 关键字确保不仅编译自己时能找到，别的包链接这个库时也能找到
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# 3. 导出目标与依赖，供其他 ROS2 包使用
ament_export_targets(${PROJECT_NAME}Targets HAS_LIBRARY_TARGET)
ament_export_dependencies()

# 4. 安装规则：将生成的动态库 .so 文件部署到 install/lib 目录
install(TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}Targets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)


install(DIRECTORY include/
  DESTINATION include
)

# ================= 新增核心逻辑结束 =================

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
