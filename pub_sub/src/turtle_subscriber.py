#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose  # 注意：turtlesim 自定义消息

class TurtleSubscriber(Node):
    def __init__(self):
        super().__init__('turtle_subscriber')
        # 创建 Subscriber：订阅 "/turtle1/pose"
        self.subscription = self.create_subscription(
            Pose,                     # 消息类型
            '/turtle1/pose',          # 话题名
            self.listener_callback,   # 回调函数
            10)                       # 队列大小
        self.subscription  # 防止被 Python 垃圾回收

    def listener_callback(self, msg):
        # msg 是 Pose 类型，包含 x, y, theta
        self.get_logger().info(f'Received pose: x={msg.x:.2f}, y={msg.y:.2f}, theta={msg.theta:.2f}')

def main(args=None):
    rclpy.init(args=args)
    node = TurtleSubscriber()
    rclpy.spin(node)      # 进入事件循环
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()