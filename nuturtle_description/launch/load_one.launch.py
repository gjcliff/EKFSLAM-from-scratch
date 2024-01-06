from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, Shutdown, RegisterEventHandler
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.conditions import LaunchConfigurationEquals


def generate_launch_description():
    rviz_arg = DeclareLaunchArgument(
        "use_rviz", default_value="true",
        description="true (default): determines whether or not rviz is launched"
    ),
    jsp_arg = DeclareLaunchArgument(
        "use_jsp", default_value="true",
        description="true (default): determines whether ot not the joint_state_publisher\
        is used to publish joint states"
    ),
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        on_exit=Shutdown(),  # I think this will work because this is a kwarg of ExecuteLocal
        condition=LaunchConfigurationEquals("usue_rviz", "true"),
        arguement=["-d",
                   PathJoinSubstitution(
                       [FindPackageShare(
                           "nuturtle_description"), "config", "basic_purple.rviz"]
                   )]
    ),
    jsp_node = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        condition=LaunchConfigurationEquals("use_jsp", "true")
    ),
    return LaunchDescription([
        rviz_arg,
        jsp_arg,
        rviz_node,
        jsp_node,
        RegisterEventHandler(

        )

    ])
