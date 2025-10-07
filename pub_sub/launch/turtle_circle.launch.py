from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # 1. 海龟仿真器
        Node(package='turtlesim', executable='turtlesim_node', name='sim'),

        # 2. C++ 画圆
        #Node(package='pub_sub', executable='turtle_publisher_cpp', name='cpp_pub'),

        # 3. C++ 打印位姿
        #Node(package='pub_sub', executable='turtle_subscriber_cpp', name='cpp_sub'),

        # 4. Python 画圆（同样发 /turtle1/cmd_vel）
        Node(package='pub_sub', executable='turtle_publisher_py', name='py_pub'),

        # 5. Python 打印位姿（同样订阅 /turtle1/pose）
        Node(package='pub_sub', executable='turtle_subscriber_py', name='py_sub'),
    ])