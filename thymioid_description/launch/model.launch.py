import os
from typing import List

import launch.actions
import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchContext, LaunchDescription
from launch.utilities import perform_substitutions
from launch_ros.actions import Node


def urdf(name: str = '') -> str:
    urdf_xacro = os.path.join(get_package_share_directory('thymioid_description'),
                              'urdf', 'thymioid.urdf.xacro')
    opts, input_file_name = xacro.process_args([urdf_xacro, f'name:={name}'])
    try:
        doc = xacro.process_file(input_file_name, **vars(opts))
    except Exception as e:
        print(e)
    return doc.toprettyxml(indent='  ')


def robot_state_publisher(context: LaunchContext, name: launch.substitutions) -> List[Node]:
    name = perform_substitutions(context, [name])
    params = {'robot_description': urdf(name)}
    node = Node(
        package='robot_state_publisher',
        node_executable='robot_state_publisher',
        node_name='robot_state_publisher',
        parameters=[params], output='screen')
    return [node]


def generate_launch_description() -> None:
    name_arg = launch.actions.DeclareLaunchArgument(
        'name', default_value='', description='The robot name')

    return LaunchDescription([
        name_arg,
        launch.actions.LogInfo(msg=launch.substitutions.LaunchConfiguration('name')),
        launch.actions.OpaqueFunction(
            function=robot_state_publisher,
            kwargs={'name': launch.substitutions.LaunchConfiguration('name')}),
    ])
