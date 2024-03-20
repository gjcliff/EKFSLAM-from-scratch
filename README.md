# Extended Kalman Filter SLAM From Scratch
# Package List
This repository consists of several ROS packages
- **nuturtle_description** - This package contains urdf files and basic debugging, testing, and visualization code for turtlebot3s.
- **nusim** - This package contains ros nodes and launchfiles that display turtlebots inside a user-defined arena with user-defined obstacles.
- **nuturtle_control** - This package enables control of the turtlebot via geometry_msgs/msg/Twist messages on the cmd_vel topic.
    - This package contains three nodes:
        * **turtle_control** - Manage the flow of information between all nodes. Convert twists into wheel command values, and motor encoder values into joint states.
        * **odometry** - Perform odometry calculations for the turtlebot, and publish them for other nodes to see
        * **circle** - Command the robot to move in a circle, and offer some additional movement related services (stopping and reversing)
- **nuslam** - This package contains a node and a launch file that performs EKF SLAM using the turtlebot.

This repository also consists of a custom c++ library:
- **turtlelib** - This library contains functions for performing geometry operations in se(2) space and displaying these operations in svg files for testing. This is a C++ library with unit tests for confirming functionality.

**To run SLAM with simulated landmark data**:  
type in the following command:
```bash
$ ros2 launch nuslam slam.launch.py
```

**To run SLAM with unknown data association from LIDAR**:
```bash
$ ros2 launch nuslam unknown_data_assoc.launch.xml
```

**To run SLAM on the turtlebot**:  
on the turtlebot, type in the following command:
```bash
$ ros2 launch nuslam turtlebot_bringup.launch.py
```  

on your computer:
```bash
$ ros2 launch nuslam pc_bringup.launch.py
```

Also check out the post on my website about this project that explains a little
more of what it's about: https://graham-clifford.com/SLAM-from-scratch/
