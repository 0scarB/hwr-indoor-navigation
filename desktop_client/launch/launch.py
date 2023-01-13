import launch
from launch.substitutions import Command, LaunchConfiguration
import launch_ros
import os

def generate_launch_description():
    pkg_share = launch_ros.substitutions.FindPackageShare(package="desktop_client2").find("desktop_client2")
    default_model_path = os.path.join(pkg_share, "src/description/pi_bot_description.urdf")
    default_rviz_config_path = os.path.join(pkg_share, "config/urdf_config.rviz")
    slam_toolbox_dir = launch_ros.substitutions.FindPackageShare(package="desktop_client2").find("slam_toolbox")
    # slam_toolbox_localization_launch_file = os.path.join(slam_toolbox_dir, "launch", "localization_launch.py")
    slam_toolbox_online_async_launch_file = os.path.join(slam_toolbox_dir, "launch", "online_async_launch.py")
    params_file = os.path.join(pkg_share, "config/nav2_params.yaml")

    robot_state_publisher_node = launch_ros.actions.Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": Command(["xacro ", LaunchConfiguration("model")])}]
    )
    joint_state_publisher_node = launch_ros.actions.Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        name="joint_state_publisher",
    )
    joint_state_publisher_gui_node = launch_ros.actions.Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        name="joint_state_publisher_gui",
        condition=launch.conditions.IfCondition(LaunchConfiguration("gui"))
    )
    rviz_node = launch_ros.actions.Node(
        package="rviz2",
        executable="rviz2",
        name="rviz",
        output="screen",
        arguments=["-d", LaunchConfiguration("rvizconfig")],
    )
    # map_server_node = launch_ros.actions.Node(
    #     package="nav2_map_server",
    #     executable="map_saver_cli",
    #     name="map_server",
    #     # arguments=["-f", "/map"],
    # )
    amcl_node = launch_ros.actions.Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[params_file]
    )

    return launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(
            name="gui",
            default_value="True",
            description="Flag to enable joint_state_publisher_gui"
        ),
        launch.actions.DeclareLaunchArgument(
            name="model",
            default_value=default_model_path,
            description="Absolute path to robot urdf file"
        ),
        launch.actions.DeclareLaunchArgument(
            name="rvizconfig",
            default_value=default_rviz_config_path,
            description="Absolute path to rviz config file"
        ),
        # launch.actions.IncludeLaunchDescription(
        #     launch.launch_description_sources.PythonLaunchDescriptionSource([slam_toolbox_localization_launch_file])
        # ),
        launch.actions.IncludeLaunchDescription(
            launch.launch_description_sources.PythonLaunchDescriptionSource(
                [slam_toolbox_online_async_launch_file],
            ),
            launch_arguments={
                "slam_params_file": os.path.join(os.path.join(pkg_share, "slam_toolbox/mapper_params_online_async.yaml"))
            }.items()
        ),
        joint_state_publisher_node,
        joint_state_publisher_gui_node,
        robot_state_publisher_node,
        rviz_node,
        # map_server_node,
        amcl_node,
        launch_ros.actions.Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_localization',
            output='screen',
            parameters=[{'autostart': True}, {'node_names': ['amcl']}],
        )
    ])