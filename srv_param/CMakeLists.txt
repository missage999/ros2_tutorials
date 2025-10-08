cmake_minimum_required(VERSION 3.8)
project(srv_param)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)

# 支持自定义服务
find_package(rosidl_default_generators REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/SetMaxVelocity.srv"
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# C++ Server
# 1. 生成可执行文件
add_executable(param_server src/param_server.cpp)

# 2. 依赖 rclcpp + 本包生成的接口
ament_target_dependencies(param_server rclcpp)

# 3. 关键：让可执行文件依赖生成的 headers 与 typesupport
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} rosidl_typesupport_cpp)
target_link_libraries(param_server "${cpp_typesupport_target}")

# 4. 把生成头文件目录挂进来
rosidl_target_interfaces(param_server ${PROJECT_NAME} "rosidl_typesupport_cpp")

# 5. 安装
install(TARGETS param_server DESTINATION lib/${PROJECT_NAME})

# Python Client
install(PROGRAMS src/param_client.py
        DESTINATION lib/${PROJECT_NAME}
        RENAME param_client)

# 配置文件 & Launch
install(DIRECTORY config
        DESTINATION share/${PROJECT_NAME}
        FILES_MATCHING PATTERN "*.yaml")
install(DIRECTORY launch/ DESTINATION share/${PROJECT_NAME}/)

ament_package()
