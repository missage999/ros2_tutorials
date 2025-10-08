from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # 声明 launch 参数：max_vel（默认 1.5）
    max_vel_arg = DeclareLaunchArgument(
        'max_vel',
        default_value='1.5',
        description='Maximum velocity to set via service'
    )

    # 获取参数文件路径
    config_file = PathJoinSubstitution([
        FindPackageShare('srv_param'),
        'config',
        'robot_params.yaml'
    ])

    # Service Server 节点
    server_node = Node(
        package='srv_param',
        executable='param_server',
        name='velocity_manager',
        parameters=[config_file],
        output='screen'
    )

    # Service Client 节点（通过 ros2 run 调用，传参）
    client_exec = ExecuteProcess(
        cmd=[
            'ros2', 'run', 'srv_param', 'param_client',
            LaunchConfiguration('max_vel')
        ],
        output='screen'
    )

    return LaunchDescription([
        max_vel_arg,
        server_node,
        client_exec
    ])