from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

def generate_launch_description():
    return LaunchDescription([
        # 启动 turtlesim_node
        Node(
            package='turtlesim',
            executable='turtlesim_node',
            name='sim'
        ),
        # 启动 C++ Publisher
        Node(
            package='pub_sub',
            executable='turtle_publisher',
            name='turtle_publisher'
        ),
        # 启动 Python Subscriber
        Node(
            package='pub_sub',
            executable='turtle_subscriber',
            name='turtle_subscriber'
        )
    ])