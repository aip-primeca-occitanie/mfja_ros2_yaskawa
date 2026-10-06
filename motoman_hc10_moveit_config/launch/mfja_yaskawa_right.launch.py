from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import PushRosNamespace
from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_demo_launch


def launch_setup(context, *args, **kwargs):
    # Retrieve boolean argument as lower-case string ("true" or "false")
    use_fake_hw = LaunchConfiguration("use_fake_hardware").perform(context).lower()

    moveit_config = (
        MoveItConfigsBuilder("yaskawa_RIGHT", package_name="motoman_hc10_moveit_config")
        .robot_description(
            mappings={
                "use_fake_hardware": use_fake_hw,
                #"prefix": "left_",
            }
        )
        .to_moveit_configs()
    )

    namespaced_demo = GroupAction([
        PushRosNamespace("yaskawa_RIGHT"),
        generate_demo_launch(moveit_config)
    ])

    return [namespaced_demo]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "use_fake_hardware",
            default_value="true",
            description="Set 'false' for real hardware, 'true' for fake hardware.",
        ),
        OpaqueFunction(function=launch_setup),
    ])