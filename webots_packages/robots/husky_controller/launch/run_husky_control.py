import os

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory, get_package_prefix

from webots_ros2_driver.webots_controller import WebotsController
from webots_ros2_driver.utils import controller_url_prefix

PACKAGE_NAME = 'husky_controll'

def generate_launch_description():
    package_dir = get_package_share_directory(PACKAGE_NAME)
    description_dir= get_package_share_directory('husky_description')
    robot_description_path = os.path.join(description_dir, 'robot', 'husky.urdf')

    # Path to model_e spawner
    husky_spawner = os.path.join(
        get_package_share_directory('husky_description'), 
        'launch', 
        'spawn_husky.py'
    )

    # Webots Driver Node (robot controller)
    webots_driver = WebotsController(
        robot_name='husky',
        parameters=[
            {'robot_description': robot_description_path},
            {'use_sim_time': True}
        ]
    )

    # # Ground Truth Publisher (Supervisor)
    ground_truth_node = Node(
        package='gt_pub',
        executable='ground_truth_publisher',
        name='ground_truth_publisher',
        output='screen',
        parameters=[
            {'robot_name': 'husky'},
            {'use_sim_time': True}
        ],
        additional_env={
            'WEBOTS_CONTROLLER_URL': controller_url_prefix(1234) + 'ground_truth_publisher',
        }
    )

    # Robot State Publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[
            {'robot_description': open(robot_description_path).read()},
            {'use_sim_time': True}
        ]
    )

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(husky_spawner)
        ),
        ground_truth_node,
        webots_driver,
        robot_state_publisher,
    ])
