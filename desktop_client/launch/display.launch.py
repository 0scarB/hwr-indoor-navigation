import os
import launch
from ament_index_python.packages import get_package_share_directory 
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
import launch_ros

def generate_launch_description():
    default_rviz_config_path = os.path.join(get_package_share_directory('desktop_client'), 'urdf_config.rviz')
    slam_toolbox_launch_path = os.path.join(get_package_share_directory('slam_toolbox'), 'launch')

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
        slam_node,
        rviz_node
    ])
