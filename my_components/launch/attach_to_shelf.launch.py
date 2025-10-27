from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():

    pre_approach = ComposableNode(
        package="my_components",
        plugin="my_components::PreApproach",
        name="pre_approach",
        namespace="",
        parameters=[{"use_sim_time": True}],
    )

    attach_server = ComposableNode(
        package="my_components",
        plugin="my_components::AttachServer",
        name="attach_server",
        namespace="",
        parameters=[{"use_sim_time": True}],
    )

    container = ComposableNodeContainer(
        name="my_container",
        namespace="", 
        package="rclcpp_components",
        executable="component_container",
        composable_node_descriptions=[
            pre_approach,
            attach_server
            ],
        output="screen",
    )

    return LaunchDescription([
        container,
    ])
