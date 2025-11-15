import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_path = get_package_share_directory('rviz_visualizer')
    urdf_dir = os.path.join(pkg_path, 'urdf/leap_hand')
    urdf_file = os.path.join(urdf_dir, 'leap_hand_right_converted.urdf')

    os.chdir(urdf_dir)

    return LaunchDescription([
        # 1. robot_state_publisher（仍然加载 urdf）
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            arguments=[urdf_file],
            output='screen',
            remappings=[('/joint_states', '/leap_hand_right/target_joint')],  # 重映射 joint_states
        ),

        # 2. joint_state_publisher（后台运行，不用 GUI）
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            output='screen',
        ),

        # 3. 启动 RViz
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', os.path.join(get_package_share_directory('rviz_visualizer'), 'rviz_config/default.rviz')],
            output='screen',
        )
    ])
