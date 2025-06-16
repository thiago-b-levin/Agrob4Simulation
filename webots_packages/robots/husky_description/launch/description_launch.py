import os
import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory
from webots_ros2_driver.urdf_spawner import URDFSpawner
from launch_ros.actions import Node


PACKAGE_NAME = 'husky_description'

def generate_launch_description():
    package_dir = get_package_share_directory(PACKAGE_NAME)
    robot_description_path = os.path.join(package_dir, 'robot', 'husky.urdf')

    # URDF Spawner: Requests Webots to spawn the URDF-based robot
    spawn_robot = URDFSpawner(
        name='husky',
        urdf_path=robot_description_path, # Path to the URDF file
        translation='-35.5 9 0.14',
        rotation='0 0 1 0',
    )

    return LaunchDescription(
        [
            spawn_robot,
        ]
    )

