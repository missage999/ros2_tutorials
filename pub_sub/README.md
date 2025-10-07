# Turtle Publisher/Subscriber Demo

## 功能
- C++ Publisher: 发布速度指令，控制小海龟画圆
- Python Subscriber: 订阅小海龟位置并打印

## 运行步骤
1. 启动 turtlesim:
   ```bash
   cd ~/ros2_ws
   rm -rf build/pub_sub install/pub_sub
   colcon build --packages-select pub_sub
   source install/setup.bash
   ros2 launch pub_sub turtle_circle.launch.py