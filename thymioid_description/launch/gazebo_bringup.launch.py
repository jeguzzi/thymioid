import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import ThisLaunchFileDir

from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node


# I'm trying to follow
# https://github.com/ros-simulation/gazebo_ros_pkgs/wiki/ROS-2-Migration:-Gazebo-ROS-Paths

def generate_launch_description():

    model = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('thymioid_description'), 'launch',
                         'model.launch.py')))

    # model = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource([ThisLaunchFileDir(), '/model2.launch.py']))

    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')),
        launch_arguments={'factory': 'true'}.items())

    spawn_entity = Node(package='gazebo_ros', node_executable='spawn_entity.py',
                        arguments=['-entity', 'thymio', '-topic', 'robot_description',
                                   '-x', '0', '-y', '0', '-z', '0'],
                        output='screen')
    return LaunchDescription([
        # spawn_entity,
        model,
        gazebo,
    ])
