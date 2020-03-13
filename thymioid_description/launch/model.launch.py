import os
from typing import List

import launch.actions
import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchContext, LaunchDescription
from launch.utilities import perform_substitutions
from launch_ros.actions import Node


def urdf(name: str = '', camera_pitch: float = 0.2618, camera_is_fixed: bool = True,
         proximity_max_range: float = 0.12, proximity_resolution: float = 0.005,
         proximity_fov: float = 0.3, publish_ground_truth: bool = False,
         ground_truth_frame_id: str = '/world', ground_truth_frame_rate: float = 30.0,
         odom_rate: float = 20.0) -> str:
    camera_joint_type = 'fixed' if camera_is_fixed in ('1', 'True', 'true') else 'revolute'
    urdf_xacro = os.path.join(get_package_share_directory('thymioid_description'),
                              'urdf', 'thymioid.urdf.xacro')
    xacro_keys = ([k for k, _ in urdf.__annotations__.items()
                   if k not in ('return', 'camera_is_fixed')] + ['camera_joint_type'])
    kwargs = dict(locals())
    xacro_args = [f'{arg_name}:={kwargs.get(arg_name)}' for arg_name in xacro_keys]
    opts, input_file_name = xacro.process_args([urdf_xacro] + xacro_args)
    try:
        doc = xacro.process_file(input_file_name, **vars(opts))
    except Exception as e:
        print(e)
    return doc.toprettyxml(indent='  ')


def robot_state_publisher(context: LaunchContext,
                          **substitutions: launch.substitutions.LaunchConfiguration
                          ) -> List[Node]:
    kwargs = {k: perform_substitutions(context, [v]) for k, v in substitutions.items()}
    params = {'robot_description': urdf(**kwargs)}
    with open('test.urdf', 'w+') as f:
        f.write(params['robot_description'])
    node = Node(
        package='robot_state_publisher',
        node_executable='robot_state_publisher',
        node_name='robot_state_publisher',
        parameters=[params], output='screen')
    return [node]


def generate_launch_description() -> None:
    arguments = [
        launch.actions.DeclareLaunchArgument(
            k, default_value=str(urdf.__defaults__[i]), description='test')
        for i, (k, _) in enumerate(urdf.__annotations__.items()) if k != 'return']
    kwargs = {k: launch.substitutions.LaunchConfiguration(k)
              for (k, _) in urdf.__annotations__.items() if k != 'return'}
    return LaunchDescription(
        arguments + [
            launch.actions.LogInfo(msg=launch.substitutions.LaunchConfiguration('camera_is_fixed')),
            launch.actions.OpaqueFunction(
                function=robot_state_publisher,
                kwargs=kwargs),
        ])
