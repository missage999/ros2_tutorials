# srv_param — Service + Parameter Demo

## 功能
- C++ 服务端 (`param_server`)：提供 `/set_max_velocity` 服务，管理 `max_velocity` 参数
- Python 客户端 (`param_client`)：调用服务设置新速度
- 支持 YAML 初始化 + Launch 一键启动

## 问题
- 文件找不到：CMake 安装 launch 目录时，应写 launch 而非 launch/（避免路径展开错误）
- 文件找不到：Python 脚本在 CMake 中安装时，需用 RENAME 去掉 .py 后缀（ROS 2 可执行文件惯例不带后缀）

## 运行
```bash
ros2 launch srv_param param_demo.launch.py max_vel:=2.0