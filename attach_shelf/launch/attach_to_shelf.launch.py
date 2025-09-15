from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node

def generate_launch_description():
    
    # ---- Declare CLI-controllable arguments (with defaults) ----
    final_arg    = LaunchConfiguration("final_approach")
    obstacle_arg = LaunchConfiguration("obstacle")
    degrees_arg  = LaunchConfiguration("degrees")

    declare_final = DeclareLaunchArgument(
        "final_approach",
        default_value="false",  
        description="true: do final approach; false: TF only"
    )
    declare_obstacle = DeclareLaunchArgument(
        "obstacle",
        default_value="0.3",
        description="Stop distance (m) before rotating"
    )
    declare_degrees  = DeclareLaunchArgument(
        "degrees",
        default_value="-90",
        description="Rotation after stop (degrees)"
    )
    
    rviz_config = PathJoinSubstitution([
        FindPackageShare("attach_shelf"),
        "rviz",
        "config.rviz"
    ])

    approach_srv = Node(
        package="attach_shelf",
        executable="approach_service_server_node",
        name="approach_server_node",
        output="screen",
        remappings=[
            ("/robot/cmd_vel", "/diffbot_base_controller/cmd_vel_unstamped"),
        ],
        parameters=[{
            "use_sim_time": True,
        }]
    )

    pre_approach = Node(
        package="attach_shelf",
        executable="pre_approach_v2_node",
        name="pre_approach_v2",
        output="screen",
        remappings=[
            ("/robot/cmd_vel", "/diffbot_base_controller/cmd_vel_unstamped"),
            ("/odom", "/diffbot_base_controller/odom"),
        ],
        parameters=[{
            "use_sim_time": True,
            "obstacle": obstacle_arg,
            "degrees": degrees_arg,
            "final_approach": final_arg,
        }]
    )
    
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config],
        parameters=[{"use_sim_time": True}]
    )

    return LaunchDescription([
        declare_final,
        declare_degrees,
        declare_obstacle,
        approach_srv,
        pre_approach,
        rviz        
    ])