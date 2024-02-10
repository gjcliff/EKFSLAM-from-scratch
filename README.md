# ME495 Sensing, Navigation and Machine Learning For Robotics
* Graham Clifford
* Winter 2024
# Package List
This repository consists of several ROS packages
- **nuturtle_description** - This package contains urdf files and basic debugging, testing, and visualization code for turtlebot3s.
- **nusim** - This package contains ros nodes and lauchfiles that display turtlebots inside a user-defined arena with user-defined obstacles.
- **nuturtle_control** - This package enables control of the turtlebot via geometry_msgs/msg/Twist messages on the cmd_vel topic.
    - This package contains three nodes:
        * **turtle_control** - Manage the flow of information between all nodes. Convert twists into wheel command values, and motor encoder values into joint states.
        * **odometry** - Perform odometry calculations for the turtlebot, and publish them for other nodes to see
        * **circle** - Command the robot to move in a circle, and offer some additional movement related services (stopping and reversing)

This repository also consists of a custom c++ library:
- **turtlelib** - This library contains functions for performing geometry operations in se(2) space and displaying these operations in svg files for testing. This is a C++ library with unit tests for confirming functionality.

Video:

https://github.com/ME495-Navigation/slam-project-gjcliff/assets/94981561/89174fed-dee4-4afa-9876-2d2973e20900
