import os
from launch import LaunchDescription
from launch.actions import GroupAction
from launch_ros.actions import PushRosNamespace
from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_demo_launch
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    
    desc_pkg = get_package_share_directory("mfja_motoman_hc10")
    moveit_pkg = get_package_share_directory("motoman_hc10_moveit_config")
    
    # -------------------------------------------------------------------------
    # Robot 1: yaskawa_LEFT
    # -------------------------------------------------------------------------
    left_config = (
        MoveItConfigsBuilder("yaskawa_LEFT", package_name="motoman_hc10_moveit_config")
        .robot_description(
            file_path="config/yaskawa_LEFT.urdf.xacro",
            #mappings={"prefix": "left_"}  # Isolate TF frames
        )
        .robot_description_semantic(file_path="config/yaskawa_LEFT.srdf")
        .to_moveit_configs()
    )

    left_group = GroupAction([
        PushRosNamespace("left_robot"),
        generate_demo_launch(left_config)
    ])


    # -------------------------------------------------------------------------
    # Robot 2: yaskawa_RIGHT
    # -------------------------------------------------------------------------
    
    right_config = (
        MoveItConfigsBuilder("yaskawa_RIGHT", package_name="motoman_hc10_moveit_config")
        .robot_description(
            file_path="config/yaskawa_RIGHT.urdf.xacro",
            #mappings={"prefix": "right_"}  # Isolate TF frames
        )
        .robot_description_semantic(file_path="config/yaskawa_RIGHT.srdf")
        .to_moveit_configs()
    )

    right_group = GroupAction([
        PushRosNamespace("right_robot"),
        generate_demo_launch(right_config)
    ])


    # -------------------------------------------------------------------------
    # Add both robots
    # -------------------------------------------------------------------------
    
    return LaunchDescription([
        left_group,
        right_group,
    ])