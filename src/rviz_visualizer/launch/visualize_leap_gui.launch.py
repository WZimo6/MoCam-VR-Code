import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_path = get_package_share_directory('rviz_visualizer')
    urdf_dir = os.path.join(pkg_path, 'urdf/leap_hand')
    urdf_file = os.path.join(urdf_dir, 'leap_hand_right_converted.urdf')

    # 👇 修改当前工作目录，确保相对路径可用
    os.chdir(urdf_dir)

    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            arguments=[urdf_file],
            output='screen',
        ),
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            arguments=[urdf_file]
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', os.path.join(get_package_share_directory('rviz_visualizer'), 'rviz_config/default.rviz')],
            output='screen',
        )
    ])
