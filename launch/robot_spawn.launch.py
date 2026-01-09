from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.actions import RegisterEventHandler, SetEnvironmentVariable
from launch.event_handlers import OnProcessExit
from launch.actions import ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Get package
    pkg_spawn_robot = get_package_share_directory('cli_spawn')
    
    # For robot_description topic
    urdf_file = os.path.join(pkg_spawn_robot, 'urdf', 'my_robot.urdf')
    with open(urdf_file, 'r') as infp:
        robot_description = infp.read()
    params = {'robot_description': robot_description, 'use_sim_time': False}
    """
    # Publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params]
    )
    """
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=['-string', robot_description,
                   '-x', '0.0',
                   '-y', '0.0',
                   '-z', '0.0',
                   '-name', 'my_robot',
                   '-allow_renaming', 'false',
                   '/world/empty/control@ros_gz_interfaces/srv/ControlWorld',
                   '/world/empty/set_pose@ros_gz_interfaces/srv/SetEnsrvtityPose'
        ],
    )
    
    spawn_cube = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-file', os.path.join(pkg_spawn_robot, 'urdf', 'cube.sdf'),
            '-x', '0.0',
            '-y', '-0.55',
            '-z', '0.1',
            '-name', 'cube'
        ],
    )
    
    # Load controller
    load_joint_state_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'joint_state_broadcaster'],
        output='screen'
    )

    load_forward_position_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'forward_position_controller'],
        output='screen'
    )

    
    return LaunchDescription([
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=spawn_robot,
                on_exit=[load_joint_state_controller],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
               target_action=load_joint_state_controller,
               on_exit=[load_forward_position_controller],
            )
        ),
        spawn_robot,
        spawn_cube,
    ])
