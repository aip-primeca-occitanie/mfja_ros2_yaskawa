import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():
    ns_arg = DeclareLaunchArgument(
        "namespace",
        default_value="yaskawa_LEFT",
        description="Namespace for the robot nodes and topics"
    )

    # # Formats to '/yaskawa_LEFT'
    ns = PythonExpression(["'/' + '", LaunchConfiguration("namespace"), "'.lstrip('/')"])

    # Load MoveIt configuration
    moveit_config = (
        MoveItConfigsBuilder("yaskawa_LEFT", package_name="motoman_hc10_moveit_config")
        # .robot_description(file_path="config/motoman_hc10.urdf.xacro")
        # .robot_description_semantic(file_path="config/motoman_hc10.srdf")
        .planning_pipelines(pipelines=["ompl"])
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .to_moveit_configs()
    )

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