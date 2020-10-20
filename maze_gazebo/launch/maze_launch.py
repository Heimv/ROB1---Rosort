import os
from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import ExecuteProcess
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    world_file_name = "challenge_maze.world"
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    world = os.path.join(get_package_share_directory('maze_gazebo'), 'worlds', world_file_name)
    launch_file_dir = os.path.join(get_package_share_directory('maze_gazebo'), 'launch')
    world_model_dir = os.path.join(get_package_share_directory('maze_gazebo'), 'model')

    return LaunchDescription([
        SetEnvironmentVariable(name="GAZEBO_MODEL_PATH", value=world_model_dir),
        ExecuteProcess(
                cmd=['gazebo', '--verbose', world, '-s', 'libgazebo_ros_init.so'],
                output='screen',
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([launch_file_dir, '/robot_state_publisher_launch.py']),
            launch_arguments={'use_sim_time': use_sim_time}.items(),
        ),
    ])
