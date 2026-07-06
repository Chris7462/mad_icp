from os.path import join

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time'
    )

    params = join(
        get_package_share_directory('mad_icp'), 'param',
        'mad_icp_params.yaml'
    )

    mad_icp_node = Node(
        package='mad_icp',
        executable='mad_icp_node',
        name='mad_icp_node',
        output='screen',
        parameters=[
            params,
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ]
    )

    return LaunchDescription([
        declare_use_sim_time,
        mad_icp_node
    ])
