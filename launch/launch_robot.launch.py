import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch.event_handlers import OnProcessStart

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    # Set the path to different files and folders.
    robot_name_in_urdf = 'agv'
    pkg_share = FindPackageShare(package='agv_ros').find('agv_ros')
    default_launch_dir = os.path.join(pkg_share, 'launch')
    default_rviz_config_path = os.path.join(pkg_share, 'rviz', 'slam_robot.rviz')
    twist_mux_config_path = os.path.join(pkg_share, 'config', 'twist_mux.yaml')
    controller_config_file = os.path.join(pkg_share, 'config', 'my_controllers.yaml')
    laser_filter_config_file = os.path.join(pkg_share, 'config', 'box_laser_filter.yaml')
    slam_config_file = os.path.join(pkg_share, 'config', 'mapper_params_online_async.yaml')

    # Launch configuration variables
    rviz_config_file = LaunchConfiguration('rviz_config_file')
    use_rviz = LaunchConfiguration('use_rviz')
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_ros2_control = LaunchConfiguration('use_ros2_control')
    use_joystick = LaunchConfiguration('use_joystick')
    slam = LaunchConfiguration('slam')

    # Declare launch arguments
    declare_rviz_config_file_cmd = DeclareLaunchArgument(
        name='rviz_config_file',
        default_value=default_rviz_config_path,
        description='Full path to RVIZ config file to use'
    )

    declare_use_rviz_cmd = DeclareLaunchArgument(
        name='use_rviz',
        default_value='True',
        description='Whether to start RVIZ'
    )

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        name='use_sim_time',
        default_value='False',
        description='Use simulation clock if true'
    )

    declare_use_ros2_control_cmd = DeclareLaunchArgument(
        name='use_ros2_control',
        default_value='True',
        description='Use ROS2 Control as controller plugin'
    )

    declare_use_joystick_cmd = DeclareLaunchArgument(
        name='use_joystick',
        default_value='False',
        description='Whether to use joystick as controller'
    )

    declare_slam_cmd = DeclareLaunchArgument(
        name='slam',
        default_value='False',
        description='Whether to run SLAM'
    )

    # Specify actions
    # Launch Robot State Publisher
    start_robot_state_publisher_cmd = IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(default_launch_dir,'rsp.launch.py')), 
                launch_arguments={'use_sim_time': use_sim_time, 'use_ros2_control': use_ros2_control}.items()
    )

    # Launch Joystick
    start_joystick_cmd = IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(default_launch_dir, 'joystick.launch.py')),
                condition=IfCondition(use_joystick)
    )

    # Launch Laser Scan
    start_laser_scan_cmd = IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(default_launch_dir, 'sllidar_s2_launch.py'))
    )

    # Launch Laser Scan Filter
    start_laser_scan_filter_cmd = IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(default_launch_dir, 'box_laser_filter.launch.py')), 
                launch_arguments={'params_file': laser_filter_config_file}.items()
    )

    # Start Twist Mux to accomodate multiple controller
    start_twist_mux_cmd = Node(
                package="twist_mux", executable="twist_mux",
                parameters=[twist_mux_config_path],
                remappings=[('/cmd_vel_out','/diff_cont/cmd_vel_unstamped')]
    )

    # Get robot URDF information from /robot_state_publisher
    robot_description = Command(['ros2 param get --hide-type /robot_state_publisher robot_description'])

    # Launch ROS2 Controller Manager
    start_controller_manager_cmd = Node(
                package="controller_manager", executable="ros2_control_node",
                parameters=[{'robot_description': robot_description}, controller_config_file]
    )

    # Delaying the launch of controller manager to wait for robot_state_publisher
    delayed_controller_manager = TimerAction(period=3.0, actions=[start_controller_manager_cmd])

    # Spawning differential drive controller
    start_diff_drive_spawner_cmd = Node(
                package="controller_manager", executable="spawner.py",
                arguments=["diff_cont"],
    )

    delayed_diff_drive_spawner = RegisterEventHandler(
                event_handler=OnProcessStart(target_action=start_controller_manager_cmd, 
                                             on_start=[start_diff_drive_spawner_cmd],)
    )

    # Spawning joint broadcaster
    start_joint_broad_spawner_cmd = Node(
                package="controller_manager", executable="spawner.py",
                arguments=["joint_broad"],
    )

    delayed_joint_broad_spawner = RegisterEventHandler(
                event_handler=OnProcessStart(target_action=start_controller_manager_cmd,
                                             on_start=[start_joint_broad_spawner_cmd],)
    )

    # Launch Slam Toolbox
    start_slam_toolbox_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(default_launch_dir, 'online_async_launch.py')),
        condition=IfCondition(slam),
        launch_arguments={'params_file': slam_config_file, 'use_sim_time': use_sim_time}.items()
    )

    # Launch RViz
    start_rviz_cmd = Node(
                condition=IfCondition(use_rviz),
                package='rviz2', executable='rviz2', name='rviz2',
                # output='screen',
                arguments=['-d', rviz_config_file]
    )

    # Create launch description and populate
    ld = LaunchDescription()

    # Declare launch options
    ld.add_action(declare_rviz_config_file_cmd)
    ld.add_action(declare_use_rviz_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_use_ros2_control_cmd)
    ld.add_action(declare_use_joystick_cmd)
    ld.add_action(declare_slam_cmd)

    # Add the actions
    ld.add_action(start_robot_state_publisher_cmd)
    ld.add_action(start_joystick_cmd)
    ld.add_action(start_laser_scan_cmd)
    ld.add_action(start_laser_scan_filter_cmd)
    ld.add_action(start_twist_mux_cmd)
    ld.add_action(delayed_controller_manager)
    ld.add_action(delayed_diff_drive_spawner)
    ld.add_action(delayed_joint_broad_spawner)
    ld.add_action(start_slam_toolbox_cmd)
    ld.add_action(start_rviz_cmd)

    return ld