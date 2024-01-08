from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PythonExpression
from launch.actions import DeclareLaunchArgument, Shutdown, SetLaunchConfiguration
from launch.substitutions import PathJoinSubstitution, Command, LaunchConfiguration, EqualsSubstitution
from launch_ros.substitutions import FindPackageShare, ExecutableInPackage
from launch.conditions import IfCondition


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "use_rviz", default_value="true",
            description="determines whether or not rviz is launched"
        ),
        DeclareLaunchArgument(
            "use_jsp", default_value="true",
            description="determines whether ot not the joint_state_publisher is used to publish joint states"
        ),
        DeclareLaunchArgument(
            "color", default_value="purple",
            description="determines the color of the turtlebot in rviz",
            choices=["red", "green", "blue", "purple"]
        ),
        SetLaunchConfiguration("rviz_file", PythonExpression(
            ["'basic_", LaunchConfiguration("color"), ".rviz'"])),

        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            namespace=LaunchConfiguration("color"),
            parameters=[
                {"robot_description":
                 Command([ExecutableInPackage("xacro", "xacro"), " ",
                          PathJoinSubstitution(
                              [FindPackageShare(
                                  "nuturtle_description"), "urdf", "turtlebot3_burger.urdf.xacro"]), " ", PythonExpression(["'color:=", LaunchConfiguration("color"), "'"])]),
                 # I tried so many things, this was hard to figure out
                 "frame_prefix": PythonExpression(["'", LaunchConfiguration('color'), "/'"])
                 }
            ],
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            namespace=LaunchConfiguration("color"),
            on_exit=Shutdown(),  # I think this will work because this is a kwarg of ExecuteLocal
            condition=IfCondition(EqualsSubstitution(
                LaunchConfiguration("use_rviz"), "true")),
            arguments=["-d",
                       PathJoinSubstitution(
                           [FindPackageShare(
                            "nuturtle_description"), "rviz", LaunchConfiguration("rviz_file")]
                       ),
                       "-f", PythonExpression(
                           ["'", LaunchConfiguration('color'), "/base_link'"])
                       ]

        ),
        Node(
            package="joint_state_publisher",
            executable="joint_state_publisher",
            namespace=LaunchConfiguration("color"),
            condition=IfCondition(EqualsSubstitution(
                LaunchConfiguration("use_jsp"), "true"))
        ),


    ])
