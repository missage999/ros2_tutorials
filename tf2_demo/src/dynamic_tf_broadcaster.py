#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
import math

class DynamicTFBroadcaster(Node):
    def __init__(self):
        super().__init__('dynamic_tf_broadcaster')
        self.tf_broadcaster = TransformBroadcaster(self)
        self.timer = self.create_timer(0.1, self.publish_transform)  # 10Hz
        self.start_time = self.get_clock().now()  # ✅ ROS 时间（Time 对象）

    def publish_transform(self):
        current_time = self.get_clock().now()
        t = (current_time - self.start_time).nanoseconds / 1e9

        radius = 0.5
        angular_vel = 0.5
        angle = angular_vel * t

        x = radius * math.cos(angle)
        y = radius * math.sin(angle)

        # 朝向 = 切线方向（车头朝运动方向）
        yaw = angle + math.pi / 2

        tfs = TransformStamped()
        tfs.header.stamp = current_time.to_msg()
        tfs.header.frame_id = 'odom'
        tfs.child_frame_id = 'base_link'

        tfs.transform.translation.x = x
        tfs.transform.translation.y = y
        tfs.transform.translation.z = 0.0

        tfs.transform.rotation.x = 0.0
        tfs.transform.rotation.y = 0.0
        tfs.transform.rotation.z = math.sin(yaw / 2)
        tfs.transform.rotation.w = math.cos(yaw / 2)

        self.tf_broadcaster.sendTransform(tfs)

def main():
    rclpy.init()
    node = DynamicTFBroadcaster()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()