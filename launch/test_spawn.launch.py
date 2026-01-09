"""
   # -----------------------Spawn multiplite robots (test)----------------------
    # Worlds path
    world_file = os.path.join(pkg_spawn_robot, 'worlds', 'empty.world')
    model_path = os.path.join(pkg_spawn_robot, 'urdf', 'my_robot.sdf')

    # Gazebo    
    gazebo = ExecuteProcess(
        cmd=['gz', 'sim', '-v', '4', world_file],
        output='screen'
    )
    # Spawn
    spawn_robot = Node(
        package='cli_spawn',
        executable='cli_spawn',
        name='cli_spawn',
        parameters=[
            {'num_robots': 1},
            {'model_path': model_path}
        ],
        output='screen'
    )
    return LaunchDescription([
        gazebo,
        spawn_robot
    ])
"""
