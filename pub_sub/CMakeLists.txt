cmake_minimum_required(VERSION 3.8)
project(pub_sub)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ---------- 依赖 ----------
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(turtlesim REQUIRED)   # 为了 Pose 消息

# ---------- C++ 节点 ----------
add_executable(turtle_publisher_cpp   src/turtle_publisher.cpp)
add_executable(turtle_subscriber_cpp  src/turtle_subscriber.cpp)
ament_target_dependencies(turtle_publisher_cpp   rclcpp geometry_msgs)
ament_target_dependencies(turtle_subscriber_cpp  rclcpp turtlesim)

# ---------- Python 节点 ----------
find_package(ament_cmake_python REQUIRED)
install(PROGRAMS
  src/turtle_publisher.py
  src/turtle_subscriber.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME turtle_publisher_py turtle_subscriber_py   # 去掉 .py 并区分名字
)

# ---------- 安装 ----------
install(TARGETS
  turtle_publisher_cpp
  turtle_subscriber_cpp
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME})

ament_package()