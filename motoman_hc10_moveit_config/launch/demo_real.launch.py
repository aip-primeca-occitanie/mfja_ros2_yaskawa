import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder
from ament_index_python.packages import get_package_share_directory


def launch_setup(context, *args, **kwargs):
    # Get robot side argument ('left' or 'right')
    robot_side = LaunchConfiguration("robot").perform(context).lower()
    side_upper = robot_side.upper()

    # Formats to 'yaskawa_RIGHT' or 'yaskawa_LEFT'
    ns = f"yaskawa_{side_upper}"

    pkg_share = get_package_share_directory("motoman_hc10_moveit_config")

    # Define explicit paths dynamically based on side argument
    urdf_path = os.path.join(pkg_share, "config", f"yaskawa_{side_upper}.urdf.xacro")
    srdf_path = os.path.join(pkg_share, "config", f"yaskawa_{side_upper}.srdf")
    kinematics_path = os.path.join(pkg_share, "config", "kinematics.yaml")
    joint_limits_path = os.path.join(pkg_share, "config", "joint_limits.yaml")
    traj_exec_path = os.path.join(pkg_share, "config", "moveit_controllers.yaml")

    moveit_config = (
        MoveItConfigsBuilder(f"yaskawa_{side_upper}", package_name="motoman_hc10_moveit_config")
        .robot_description(file_path=urdf_path)
        .robot_description_semantic(file_path=srdf_path)
        .robot_description_kinematics(file_path=kinematics_path)
        .joint_limits(file_path=joint_limits_path)
        .trajectory_execution(file_path=traj_exec_path)
        .planning_pipelines(pipelines=["ompl"])
        .to_moveit_configs()
    )

    # Convert configs to dict and inject move_group_namespace explicitly
    moveit_config_dict = moveit_config.to_dict()
    moveit_config_dict.update({"move_group_namespace": f"/{ns}"})

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
            {"move_group_namespace": f"/{ns}"},  # Explicitly force RViz MoveGroup NS
            {"use_sim_time": False},
        ],
        remappings=[
            ("/tf", "tf"),
            ("/tf_static", "tf_static"),
        ],
    )

    return [
        robot_state_publisher_node,
        move_group_node,
        rviz_node,
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "robot",
            default_value="right",
            description="Robot side to launch: 'left' or 'right'",
            choices=["left", "right"],
        ),
        OpaqueFunction(function=launch_setup),
    ])