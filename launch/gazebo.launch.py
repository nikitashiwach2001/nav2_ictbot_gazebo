import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg = get_package_share_directory('ict_bot_nav')

    urdf_file = os.path.join(pkg, 'urdf', 'ict_bot.urdf')
    world_file = os.path.join(pkg, 'worlds', 'closed_room.world')

    with open(urdf_file, 'r') as f:
        robot_description = f.read()

    robot_description = robot_description.replace(
        'package://ict_bot_nav',
        'file://' + pkg
    )

    return LaunchDescription([

        # Launch Gazebo
        ExecuteProcess(
            cmd=[
                'gazebo', '--verbose', world_file,
                '-s', 'libgazebo_ros_init.so',
                '-s', 'libgazebo_ros_factory.so'
            ],
            output='screen'
        ),

        # Robot State Publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': robot_description,
                'use_sim_time': True
            }]
        ),

        # Joint State Publisher — COMMENTED OUT because it conflicts with
        # Gazebo's diff_drive plugin (both publish wheel joint states,
        # causing TF flickering / "hopping" in RViz).
        # The diff_drive plugin already publishes joint states on /joint_states.
        # Node(
        #     package='joint_state_publisher',
        #     executable='joint_state_publisher',
        #     name='joint_state_publisher',
        #     parameters=[{
        #         'use_sim_time': True
        #     }]
        # ),

        # Spawn Robot
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            name='spawn_ict_bot',
            arguments=[
                '-topic', 'robot_description',
                '-entity', 'ict_bot',
                '-x', '0.0',
                '-y', '0.0',
                '-z', '0.03'
            ],
            output='screen'
        ),
    ])