# tf2_demo — TF2 坐标变换系统

## 功能
- `static_tf_broadcaster.cpp`：发布 base_link → laser 静态变换
- `dynamic_tf_broadcaster.py`：模拟小车移动，发布 odom → base_link 动态变换
- `tf2_listener.cpp`：查询 laser 在 odom 坐标系下的位置
- `rviz/tf_demo.rviz`：可视化 TF 坐标系树

## 难点
- 在cmake文件中安装directory的时候，不用“/”，比如launch和rviz而不是launch/和rviz/

## 运行步骤
```bash
# 终端1
ros2 run tf2_demo static_tf_broadcaster

# 终端2
ros2 run tf2_demo dynamic_tf_broadcaster

# 终端3
ros2 run tf2_demo tf2_listener

# 终端4
rviz2 -d $(ros2 pkg prefix tf2_demo)/share/tf2_demo/rviz/tf_demo.rviz

# launch文件一键启动
# 构建
cd ~/ros2_ws
colcon build --packages-select tf2_demo
source install/setup.bash

# 运行
ros2 launch tf2_demo tf2_demo.launch.py