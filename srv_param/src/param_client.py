#!/usr/bin/env python3
import sys
import rclpy
from rclpy.node import Node
from srv_param.srv import SetMaxVelocity

class VelocityClient(Node):
    def __init__(self):
        super().__init__('velocity_client')
        self.cli = self.create_client(SetMaxVelocity, '/set_max_velocity')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting...')

    def send_request(self, max_vel):
        req = SetMaxVelocity.Request()
        req.max_vel = max_vel
        future = self.cli.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        return future.result()

def main():
    if len(sys.argv) != 2:
        print("Usage: ros2 run srv_param param_client <max_velocity>")
        return

    try:
        max_vel = float(sys.argv[1])
    except ValueError:
        print("Error: max_velocity must be a number")
        return

    rclpy.init()
    client = VelocityClient()
    response = client.send_request(max_vel)

    if response.success:
        client.get_logger().info('SUCCESS: %s' % response.message)
    else:
        client.get_logger().error('FAILED: %s' % response.message)

    client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()