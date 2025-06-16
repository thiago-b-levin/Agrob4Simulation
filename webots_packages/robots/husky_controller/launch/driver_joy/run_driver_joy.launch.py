import launch
import launch_ros.actions
import os

from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    joy_config = launch.substitutions.LaunchConfiguration('joy_config')
    joy_dev = launch.substitutions.LaunchConfiguration('joy_dev')
    config_filepath = launch.substitutions.LaunchConfiguration('config_filepath')

    dirname, filename = os.path.split(os.path.realpath(__file__))
    params = os.path.join(dirname, 'driver_teleop.yaml')

    return launch.LaunchDescription([

        #launch.actions.DeclareLaunchArgument('joy_vel', default_value='/husky_velocity_controller/cmd_vel_unstamped'),
        launch.actions.DeclareLaunchArgument('joy_vel', default_value='/cmd_vel'),
        launch.actions.DeclareLaunchArgument('joy_config', default_value='ps4'),
        launch.actions.DeclareLaunchArgument('joy_dev', default_value='/dev/input/js1'),
        launch.actions.DeclareLaunchArgument('config_filepath', default_value=[params]),
        launch_ros.actions.Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            parameters=[{
                'dev': joy_dev,
                'deadzone': 0.3,
                'autorepeat_rate': 20.0,
                'use_sim_time': True
            }]),

        launch_ros.actions.Node(
            package='teleop_twist_joy', executable='teleop_node',
            name='teleop_twist_joy_node', parameters=[config_filepath],
            remappings={('/cmd_vel', launch.substitutions.LaunchConfiguration('joy_vel'))},
            ),
    ])

