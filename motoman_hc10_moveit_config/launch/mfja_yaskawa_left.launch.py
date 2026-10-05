from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import PushRosNamespace
from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_demo_launch


def launch_setup(context, *args, **kwargs):
    # Evaluate the LaunchConfiguration to a string at launch runtime
    use_fake_hardware = LaunchConfiguration("use_fake_hardware").perform(context)

    moveit_config = (
        MoveItConfigsBuilder("yaskawa_LEFT", package_name="motoman_hc10_moveit_config")
        .robot_description(
            mappings={
                "use_fake_hardware": use_fake_hardware,
                #"prefix": "left_",
            }
        )
        .to_moveit_configs()
    )

    namespaced_demo = GroupAction([
        PushRosNamespace("left_robot"),
        generate_demo_launch(moveit_config)
    ])

    return [namespaced_demo]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "use_fake_hardware",
            default_value="false",
            description="Set to 'true' for fake hardware simulation, or 'false' for real hardware.",
        ),
        OpaqueFunction(function=launch_setup),
    ])