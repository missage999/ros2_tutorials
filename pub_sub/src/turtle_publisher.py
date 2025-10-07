#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist   # 等价于 C++ 的 geometry_msgs/msg/twist.hpp

class TurtlePublisher(Node):
    def __init__(self):
        super().__init__('turtle_publisher')          # 节点名

        # 创建 Publisher：话题 /turtle1/cmd_vel，队列 10
        self.publisher_ = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)

        # 创建定时器：每 500 ms 触发一次 timer_callback
        self.timer = self.create_timer(0.5, self.timer_callback)

    def timer_callback(self):
        msg = Twist()
        msg.linear.x = 2.0   # 前进速度
        msg.angular.z = 1.0  # 旋转速度 → 画圆！

        self.get_logger().info(f'Publishing: linear={msg.linear.x:.2f}, angular={msg.angular.z:.2f}')
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)                    # 初始化 ROS 2
    node = TurtlePublisher()
    rclpy.spin(node)                         # 保持节点运行，直到 Ctrl-C
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()