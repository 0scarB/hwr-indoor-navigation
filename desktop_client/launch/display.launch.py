import os
import pathlib
import launch
from ament_index_python.packages import get_package_share_directory 
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
import launch_ros
from pathlib import Path

def generate_launch_description():
    default_rviz_config_path = os.path.join(get_package_share_directory('desktop_client'), 'urdf_config.rviz')
    slam_toolbox_launch_path = os.path.join(get_package_share_directory('slam_toolbox'), 'launch')

    declare_params_file_cmd = launch.actions.DeclareLaunchArgument(
        'params_file',
        default_value=str(find_parent('hwr_indoor_navigation') / 'params' / 'nav2_params.yaml'),
        description='Full path to the ROS2 parameters file to use for all launched nodes'
    )
    slam_node = launch.actions.IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(slam_toolbox_launch_path, 'online_async_launch.py')
        )
    )
    rviz_node = launch_ros.actions.Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rvizconfig')],
    )

    return launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(name='rvizconfig', default_value=default_rviz_config_path,
                                            description='Absolute path to rviz config file'),
        declare_params_file_cmd,
        slam_node,
        rviz_node
    ])

def find_parent(parent_dir_name: str) -> Path:
    path = pathlib.Path(__file__).resolve()
    while True:
        if path.is_dir() and path.name == parent_dir_name:
            return path

        path = path.parent
