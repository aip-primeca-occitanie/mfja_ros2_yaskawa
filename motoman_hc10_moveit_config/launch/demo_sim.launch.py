import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def launch_setup(context, *args, **kwargs):
    # Resolve the launch argument string ('left' or 'right')
    side = LaunchConfiguration("robot").perform(context).lower()
    side_upper = side.upper()

    pkg_share = get_package_share_directory("motoman_hc10_moveit_config")

    # Define paths dynamically based on requested side
    urdf_path = os.path.join(pkg_share, "config", f"yaskawa_{side_upper}.urdf.xacro")
    srdf_path = os.path.join(pkg_share, "config", f"yaskawa_{side_upper}.srdf")
    kinematics_path = os.path.join(pkg_share, "config", "kinematics.yaml")
    joint_limits_path = os.path.join(pkg_share, "config", "joint_limits.yaml")
    controllers_path = os.path.join(pkg_share, "config", f"moveit_controllers.yaml")
    rviz_config_path = os.path.join(pkg_share, "config", "moveit.rviz")

    # Build MoveIt configs
    moveit_config = (
        MoveItConfigsBuilder(f"yaskawa_{side_upper}", package_name="motoman_hc10_moveit_config")
        .robot_description(file_path=urdf_path)
        .robot_description_semantic(file_path=srdf_path)
        .robot_description_kinematics(file_path=kinematics_path)
        .joint_limits(file_path=joint_limits_path)
        .trajectory_execution(file_path=controllers_path)
        .to_moveit_configs()
    )

    # 1. Robot State Publisher (Publishes TFs based on /joint_states)
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[moveit_config.robot_description],
    )

    # 1. ros2_control node with ros2_controllers_RIGHT.yaml
    ros2_controllers_path = os.path.join(
        pkg_share, "config", f"ros2_controllers.yaml"
    )

    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            moveit_config.robot_description,
            ros2_controllers_path,
        ],
        output="screen",
    )

    # 2. Spawner for joint states
    jsb_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        output="screen",
    )

    # 3. Spawner for trajectory controller (Creates /dn2p1_controller/follow_joint_trajectory)
    arm_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["dn2p1_controller", "--controller-manager", "/controller_manager"],
        output="screen",
    )

    # 3. MoveGroup Node (Core planner with internal fake execution)
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            {"use_sim_time": False},
        ],
    )

    # 4. RViz2 Node
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        output="screen",
        arguments=["-d", rviz_config_path],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.joint_limits,
        ],
    )

    return [
        robot_state_publisher_node,
        move_group_node,
        rviz_node,
        ros2_control_node,
        jsb_spawner,
        arm_spawner,
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "robot",
            default_value="left",
            description="Robot side to launch: 'left' or 'right'",
            choices=["left", "right"],
        ),
        OpaqueFunction(function=launch_setup),
    ])