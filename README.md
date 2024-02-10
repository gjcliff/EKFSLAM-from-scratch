# ME495 Sensing, Navigation and Machine Learning For Robotics
* Graham Clifford
* Winter 2024
# Package List
This repository consists of several ROS packages
- **nuturtle_description** - This package contains urdf files and basic debugging, testing, and visualization code for turtlebot3s.
- **nusim** - This package contains ros nodes and lauchfiles that display turtlebots inside a user-defined arena with user-defined obstacles.
- **turtlelib** - This library contains functions for performing geometry operations in se(2) space and displaying these operations in svg files for testing. This is a C++ library with unit tests for confirming functionality.
# How to Run
In your main computer terminal:
```console
$ ros2 launch nuturtle_control start_robot.launch.xml use_rviz:=true robot:=none cmd_src:=circle
```

In your turtlebot3 terminal:
```console
$ ros2 launch nuturtle_control start_robot.launch.xml use_rviz:=false robot:=localhost
```

Video:
![video](https://github.com/ME495-Navigation/slam-project-gjcliff/issues/1#issue-2128276540)