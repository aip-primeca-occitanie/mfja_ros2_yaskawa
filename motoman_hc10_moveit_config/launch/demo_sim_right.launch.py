import os
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_demo_launch


def generate_launch_description():
    # Base path for your package
    pkg_share = get_package_share_directory("motoman_hc10_moveit_config")

    # Define explicit paths to your config files
    urdf_path = os.path.join(pkg_share, "config", "yaskawa_RIGHT.urdf.xacro")
    srdf_path = os.path.join(pkg_share, "config", "yaskawa_RIGHT.srdf")
    kinematics_path = os.path.join(pkg_share, "config", "kinematics.yaml")
    joint_limits_path = os.path.join(pkg_share, "config", "joint_limits.yaml")
    traj_exec_path = os.path.join(pkg_share, "config", "moveit_controllers_RIGHT.yaml")

    moveit_config = (
        MoveItConfigsBuilder("yaskawa_RIGHT", package_name="motoman_hc10_moveit_config")
        .robot_description(file_path=urdf_path)
        .robot_description_semantic(file_path=srdf_path)
        .robot_description_kinematics(file_path=kinematics_path)
        .joint_limits(file_path=joint_limits_path)
        .trajectory_execution(file_path=traj_exec_path)
        .to_moveit_configs()
    )

    return generate_demo_launch(moveit_config)