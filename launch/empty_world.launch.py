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
    # Launch Arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default=True)
    # Get package
    pkg_spawn_robot = get_package_share_directory('cli_spawn')
    
    # Gazebo sim resource path
    gazebo_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=[
            os.path.join(pkg_spawn_robot, 'worlds')]
    )
    
    gazebo = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')]),
                #launch_arguments=[('gz_args', [os.path.join(pkg_spawn_robot, 'worlds', 'empty.world'),' -v 4', '-r'])]
                launch_arguments={'gz_args': f"{os.path.join(pkg_spawn_robot, 'worlds', 'empty.world')} -v 4 -r"}.items()
    )

    # For robot_description topic
    urdf_file = os.path.join(pkg_spawn_robot, 'urdf', 'my_robot.urdf')
    
    with open(urdf_file, 'r') as infp:
        robot_description = infp.read()
    params = {'robot_description': robot_description, 'use_sim_time': False}

    # Publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params]
    )
    # World and robot sdf file (not use)
    world_file = os.path.join(pkg_spawn_robot, 'worlds', 'empty.world')
    robot_file = os.path.join(pkg_spawn_robot, 'urdf', 'my_robot.sdf')

    """
    # Run gazebo
    gazebo = ExecuteProcess(
        cmd = ['gz', 'sim', '-v', '4', world_file],
        output = 'screen'
    )
    # Spawn
    spawn_robot = Node(
        package='cli_spawn',
        executable='cli_spawn',
        parameters=[
            {'num_robots': 3},
            {'model_path': robot_file}
        ],
        output='screen'
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

    # Bridge
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan'],
        output='screen'
    )
    
    # Add joint_state_bridge and tf_bridge
    """
    joint_state_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            "/world/empty/model/my_robot/joint_state@sensor_msgs/msg/JointState[gz.msgs.Model"
        ],
        output='screen'
    )
    """
    
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
        gazebo_resource_path,
        gazebo,
        robot_state_publisher,
        spawn_robot,
        spawn_cube,
        bridge,
    ])
