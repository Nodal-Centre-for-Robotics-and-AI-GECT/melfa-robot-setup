from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
import os
from ament_index_python.packages import get_package_share_directory
from launch.actions import TimerAction
import yaml

# Paths to the other launch files using package share path
start_controllers = os.path.join(get_package_share_directory('melfa_bringup'), 'launch', 'rv5as_control.launch.py')
start_moveit = os.path.join(get_package_share_directory('melfa_rv5as_moveit_config'), 'launch', 'rv5as_moveit.launch.py')


def generate_launch_description():
    # Declare arguments that can be passed to start_controllers
    declared_arguments = []
    
    declared_arguments.append(
        DeclareLaunchArgument(
            'use_fake_hardware',
            default_value='true',
            description='Start robot with fake hardware mirroring command to its states.',
        )
    )
    

    # # Path to the YAML config file
    # config_file = os.path.join(get_package_share_directory('melfa_examples'), 'config', 'waypoints.yaml')

    # # Parse the YAML file
    # with open(config_file, 'r') as f:
    #     waypoints_config = yaml.safe_load(f)

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(start_controllers),
            launch_arguments={
                'use_fake_hardware': 'true',

            }.items()
        ),
        TimerAction(
            period=2.0,
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(start_moveit),
                    launch_arguments={
                        'start_rviz': 'true',
                    }.items()
                )
            ]
        ),

        TimerAction(
            period=10.0,
            actions=[
                Node(
                    package='melfa_examples',
                    executable='melfa_loop_waypoints',
                    name='melfa_waypoints_node',
                    output='screen',
                    # parameters=[
                    #     {'waypoints': waypoints_config}
                    # ]
                )
            ]
        )
    ])
