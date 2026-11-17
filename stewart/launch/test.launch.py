import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, AppendEnvironmentVariable,
                            IncludeLaunchDescription)
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    pkg_stewart = get_package_share_directory('stewart')
    gz_launch_path = PathJoinSubstitution([pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py'])
    gz_model_path = os.path.join(pkg_stewart, 'models')

    return LaunchDescription([
        # world name without .sdf (e.g. empty, or any world on GZ_SIM_RESOURCE_PATH)
        DeclareLaunchArgument('world', default_value='empty'),
        AppendEnvironmentVariable('GZ_SIM_RESOURCE_PATH', gz_model_path),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gz_launch_path),
            launch_arguments={
                'gz_args': [LaunchConfiguration('world'), '.sdf'],
                'on_exit_shutdown': 'True'
            }.items(),
        ),
    ])
