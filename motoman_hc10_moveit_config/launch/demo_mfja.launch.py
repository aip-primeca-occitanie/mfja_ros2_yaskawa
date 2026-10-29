import os
import launch.logging
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder
from ament_index_python.packages import get_package_share_directory


def launch_setup(context, *args, **kwargs):
    logger = launch.logging.get_logger("demo_mfja.launch.py")
    
    # Resolve the launch argument "robor"" ('left' or 'right')
    ROBOT_SIDE = LaunchConfiguration("robot").perform(context).upper()
    # Resolve the launch argument "hardware" ('real' or 'sim')
    hardware = LaunchConfiguration("hardware").perform(context).upper()
    REAL_ROBOT = (hardware == "REAL")
    
    # Namespace logic: Root '/' for sim, namespaced ('yaskawa_RIGHT' or 'yaskawa_LEFT') for real hardware
    ns = f"yaskawa_{ROBOT_SIDE}" if REAL_ROBOT else None

    logger.info(f"Starting robot bringup for {ROBOT_SIDE} side, {hardware} robot")
    pkg_share = get_package_share_directory("motoman_hc10_moveit_config")

    # Define paths dynamically based on requested side (left or right)
    urdf_path = os.path.join(pkg_share, "config", f"yaskawa_{ROBOT_SIDE}.urdf.xacro")
    srdf_path = os.path.join(pkg_share, "config", f"yaskawa_{ROBOT_SIDE}.srdf")
    kinematics_path = os.path.join(pkg_share, "config", "kinematics.yaml")
    joint_limits_path = os.path.join(pkg_share, "config", "joint_limits.yaml")
    controllers_path = os.path.join(pkg_share, "config", "moveit_controllers.yaml")
    rviz_config_path = os.path.join(pkg_share, "config", f"moveit_{ROBOT_SIDE}.rviz")

    # Build MoveIt configs
    moveit_config = (
        MoveItConfigsBuilder(f"yaskawa_{ROBOT_SIDE}", package_name="motoman_hc10_moveit_config")
        .robot_description(file_path=urdf_path)
        .robot_description_semantic(file_path=srdf_path)
        .robot_description_kinematics(file_path=kinematics_path)
        .joint_limits(file_path=joint_limits_path)
        .trajectory_execution(file_path=controllers_path)
        .planning_pipelines(
            pipelines=["ompl"],
            default_planning_pipeline="ompl",
        )
        .to_moveit_configs()
    )

    # Configure dictionaries and remappings conditionally
    moveit_config_dict = moveit_config.to_dict()
    remappings = [
        ("/tf", "tf"),
        ("/tf_static", "tf_static"),
        ("tf", f"/{ns}/tf" if REAL_ROBOT else "tf"),
        ("tf_static", f"/{ns}/tf_static" if REAL_ROBOT else "tf_static"),
        ("move_action", f"/{ns}/move_action") if REAL_ROBOT else "move_action",
    ]

    if REAL_ROBOT:
        moveit_config_dict.update({"move_group_namespace": f"/{ns}"})


    # 1. Robot State Publisher (Publishes TFs based on /joint_states)
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        namespace=ns,
        output="screen",
        parameters=[moveit_config.robot_description],
        remappings=remappings,
    )

    # 2. MoveGroup Node
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        namespace=ns,
        output="both",
        parameters=[
            moveit_config_dict,
            {"use_sim_time": False},
        ],
    )

    # 3. RViz Node
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        namespace=ns,
        output="screen",
        arguments=["-d", rviz_config_path],
        parameters=[
            moveit_config_dict,
            # {"move_group_namespace": f"/{ns}"} if REAL_ROBOT else {},  # Explicitly force RViz MoveGroup NS
            {"use_sim_time": False},
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            # moveit_config.planning_pipelines,
            moveit_config.joint_limits,
        ],
        remappings=remappings,
    )

    nodes = [
            robot_state_publisher_node,
            move_group_node,
            rviz_node,
    ]

    # Create simulation-only nodes
    if not REAL_ROBOT:
        # 4. ros2_control node with ros2_controllers_RIGHT.yaml
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

        # 5. Spawner for joint states
        jsb_spawner = Node(
            package="controller_manager",
            executable="spawner",
            arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
            output="screen",
        )

        # 6. Spawner for trajectory controller (Creates /dn2p1_controller/follow_joint_trajectory)
        arm_spawner = Node(
            package="controller_manager",
            executable="spawner",
            arguments=["dn2p1_controller", "--controller-manager", "/controller_manager"],
            output="screen",
        )
        
        nodes.extend([
            ros2_control_node,
            jsb_spawner,
            arm_spawner,
        ])

    # Return all created nodes
    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "robot",
            default_value="left",
            description="Robot side to launch: 'left' or 'right'",
            choices=["left", "right"],
        ),
        DeclareLaunchArgument(
            "hardware",
            default_value="sim",
            description="Wether real or simulated hardware is used: 'real' or 'sim'",
            choices=["real", "sim"],
        ),
        OpaqueFunction(function=launch_setup),
    ])