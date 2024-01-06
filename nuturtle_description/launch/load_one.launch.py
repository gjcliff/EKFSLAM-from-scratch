from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, Shutdown
from launch.substitutions import PathJoinSubstitution, Command
from launch_ros.substitutions import FindPackageShare, ExecutableInPackage
from launch.conditions import LaunchConfigurationEquals


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "use_rviz", default_value="true",
            description="true (default): determines whether or not rviz is launched"
        ),
        DeclareLaunchArgument(
            "use_jsp", default_value="true",
            description="true (default): determines whether ot not the joint_state_publisher\
            is used to publish joint states"
        ),
        Node(
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
        Node(
            package="joint_state_publisher",
            executable="joint_state_publisher",
            condition=LaunchConfigurationEquals("use_jsp", "true")
        ),
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            parameters=[
                {"robot_description":
                 Command([ExecutableInPackage("xacro", "xacro"), " ",
                          PathJoinSubstitution(
                              [FindPackageShare(
                                  "nuturtle_description"), "turtlebot3_burger.urdf.xacro"])])}
            ]
        )

    ])
