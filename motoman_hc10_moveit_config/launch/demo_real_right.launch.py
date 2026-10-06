import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    ns_arg = DeclareLaunchArgument(
        "namespace",
        default_value="yaskawa_RIGHT",
        description="Namespace for the robot nodes and topics"
    )

    # # Formats to '/yaskawa_RIGHT'
    ns = PythonExpression(["'/' + '", LaunchConfiguration("namespace"), "'.lstrip('/')"])

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
        .planning_pipelines(pipelines=["ompl"])
        .to_moveit_configs()
    )


    # # Load MoveIt configuration
    # moveit_config = (
    #     MoveItConfigsBuilder("yaskawa_RIGHT", package_name="motoman_hc10_moveit_config")
    #     # .robot_description(file_path="config/motoman_hc10.urdf.xacro")
    #     # .robot_description_semantic(file_path="config/motoman_hc10.srdf")
    #     .planning_pipelines(pipelines=["ompl"])
    #     .trajectory_execution(file_path="config/moveit_controllers_RIGHT.yaml")
    #     .to_moveit_configs()
    # )

    # Convert configs to dict and inject move_group_namespace explicitly
    moveit_config_dict = moveit_config.to_dict()
    moveit_config_dict.update({"move_group_namespace": ns})

    # 1. Robot State Publisher
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        namespace=ns,
        output="screen",
        parameters=[moveit_config.robot_description],
    )

    # 2. MoveGroup Node
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        namespace=ns,
        output="screen",
        parameters=[moveit_config_dict],
    )

# 3. RViz Node
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        namespace=ns,
        output="screen",
        arguments=["-d", os.path.join(
            moveit_config.package_path, "config", "moveit.rviz"
        )],
        parameters=[
            moveit_config.to_dict(),
            {"move_group_namespace": ns},  # Explicitly force RViz MoveGroup NS
            {"use_sim_time": False},
        ],
        remappings=[
            ("/tf", "tf"),
            ("/tf_static", "tf_static"),
        ],
    )

    return LaunchDescription([
        ns_arg,
        robot_state_publisher_node,
        move_group_node,
        rviz_node,
    ])