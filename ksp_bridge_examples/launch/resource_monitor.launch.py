from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    ld.add_action(
        Node(
            package="ksp_bridge",
            executable="ksp_bridge",
        )
    )

    ld.add_action(
        Node(
            package="ksp_bridge_examples",
            executable="kspb_resource_monitor",
        )
    )

    return ld
