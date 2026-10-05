from launch import LaunchDescription
from launch.actions import GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import PushRosNamespace
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_share = get_package_share_directory('motoman_hc10_moveit_config')
    demo_launch = os.path.join(pkg_share, 'launch', 'demo.launch.py')

    # Robot 1 setup
    yaskawa_right_group = GroupAction([
        PushRosNamespace('yaskawa_RIGHT'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(demo_launch),
            launch_arguments={
                'use_fake_hardware': 'true',  # or 'false'
                'tf_prefix': 'yaskawa_RIGHT_',
            }.items()
        )
    ])

    return LaunchDescription([
        yaskawa_right_group,
    ])