import os
import pathlib
from dataclasses import dataclass

import launch
from ament_index_python.packages import get_package_share_directory 
from launch.launch_description_sources import PythonLaunchDescriptionSource, AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
import launch_ros
from pathlib import Path


@dataclass(frozen=True)
class Paths:
    project_root: Path
    nav2_params_file: Path
    default_rviz_config_file: Path
    slam_toolbox_launch_file: Path
    costmap_2d_launch_file: Path


def generate_launch_description():
    run_tests()

    paths = create_paths()

    declare_params_file_cmd = launch.actions.DeclareLaunchArgument(
        'params_file',
        default_value=str(paths.nav2_params_file),
        description='Full path to the ROS2 parameters file to use for all launched nodes'
    )
    slam_node = launch.actions.IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(paths.slam_toolbox_launch_file))
    )
    rviz_node = launch_ros.actions.Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rvizconfig')],
    )
    amcl_node = launch_ros.actions.Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
    )
    costmap_2d_node = launch.actions.IncludeLaunchDescription(
        AnyLaunchDescriptionSource("./costmap_2d.launch")
    )

    return launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(name='rvizconfig', default_value=str(paths.default_rviz_config_file),
                                            description='Absolute path to rviz config file'),
        declare_params_file_cmd,
        slam_node,
        rviz_node,
        amcl_node,
        costmap_2d_node
    ])


def create_paths() -> Paths:
    project_root = find_parent('hwr-indoor-navigation')
    nav2_params_file = project_root / 'params' / 'nav2_params.yaml'
    default_rviz_config_file = (Path(get_package_share_directory('desktop_client')) / 'urdf_config.rviz').resolve()
    slam_toolbox_launch_dir = (Path(get_package_share_directory('slam_toolbox')) / 'launch').resolve()
    slam_toolbox_launch_file = slam_toolbox_launch_dir / 'online_async_launch.py'

    return Paths(
        project_root=project_root,
        nav2_params_file=nav2_params_file,
        default_rviz_config_file=default_rviz_config_file,
        slam_toolbox_launch_file=slam_toolbox_launch_file,
    )


def find_parent(parent_dir_name: str) -> Path:
    path = pathlib.Path(__file__).resolve()
    while True:
        if path.is_dir() and path.name == parent_dir_name:
            return path

        path = path.parent


def run_tests():
    def test_paths():
        paths = create_paths()

        assert paths.nav2_params_file.exists()
        assert paths.nav2_params_file.is_file()

        assert paths.default_rviz_config_file.is_file()

        assert paths.slam_toolbox_launch_file.is_file()

    for test in [
        test_paths,
    ]:
        test()

