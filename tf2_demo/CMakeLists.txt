cmake_minimum_required(VERSION 3.8)
project(tf2_demo)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ==============================
# 🔑 关键修正：添加 tf2_geometry_msgs
# ==============================
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)             # 虽未直接用于C++，但包依赖中声明了
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_msgs REQUIRED)          # 实际可省略（除非直接用TFMessage）
find_package(tf2_geometry_msgs REQUIRED) # ✅ 必须添加！用于 getYaw / doTransform
find_package(geometry_msgs REQUIRED)

# ==============================
# 测试配置（可选）
# ==============================
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # 跳过版权检查（开发阶段可接受）
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# ==============================
# 可执行文件：C++ 节点
# ==============================

# 静态 TF 广播器
add_executable(static_tf_broadcaster src/static_tf_broadcaster.cpp)
ament_target_dependencies(static_tf_broadcaster
  rclcpp
  tf2_ros
  geometry_msgs
)
install(TARGETS static_tf_broadcaster
  DESTINATION lib/${PROJECT_NAME}
)

# TF 监听器（使用 tf2_geometry_msgs）
add_executable(tf2_listener src/tf2_listener.cpp)
ament_target_dependencies(tf2_listener
  rclcpp
  tf2_ros
  tf2_geometry_msgs  # ✅ 正确链接
)
install(TARGETS tf2_listener
  DESTINATION lib/${PROJECT_NAME}
)

# ==============================
# Python 脚本 & 配置文件
# ==============================

# 动态 TF 广播器（Python）
install(PROGRAMS src/dynamic_tf_broadcaster.py
  DESTINATION lib/${PROJECT_NAME}
)

# RViz 配置文件
install(DIRECTORY rviz
  DESTINATION share/${PROJECT_NAME}/
)

# Launch 文件
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

# ==============================
# 最终声明
# ==============================
ament_package()