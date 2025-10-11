from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # 查找 RViz 配置文件路径
    rviz_config_path = PathJoinSubstitution(
        [FindPackageShare("tf2_demo"), "rviz", "tf_demo.rviz"]
    )

    # 定义 RViz 节点（稍后通过 TimerAction 延迟启动）
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_path],
        output='screen'
    )

    return LaunchDescription([
        # 静态 TF 广播器 (C++)
        Node(
            package='tf2_demo',
            executable='static_tf_broadcaster',
            name='static_tf_broadcaster',
            output='screen'
        ),

        # 动态 TF 广播器 (Python)
        Node(
            package='tf2_demo',
            executable='dynamic_tf_broadcaster.py',
            name='dynamic_tf_broadcaster',
            output='screen'
        ),

        # TF 监听器 (C++)
        Node(
            package='tf2_demo',
            executable='tf2_listener',
            name='tf2_listener',
            output='screen'
        ),

        # 延迟 1 秒启动 RViz，确保 TF 已发布
        TimerAction(
            period=1.0,
            actions=[rviz_node]
        ),
    ])