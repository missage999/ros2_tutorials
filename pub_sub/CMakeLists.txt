cmake_minimum_required(VERSION 3.8)
project(pub_sub)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

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

# 添加可执行文件：turtle_publisher
add_executable(turtle_publisher src/turtle_publisher.cpp)
# 链接依赖库
ament_target_dependencies(turtle_publisher rclcpp geometry_msgs)

# 安装可执行文件（重要！否则 ros2 run 找不到）
install(TARGETS
  turtle_publisher
  DESTINATION lib/${PROJECT_NAME})

  # 安装 launch 文件
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}
)

# 找到 ament_cmake_python
find_package(ament_cmake_python REQUIRED)

install(PROGRAMS
  src/turtle_subscriber.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME turtle_subscriber          # <-- 加这一行
)

ament_package()
