Author: Graham Clifford

### Package Description:
This package creates an environment in rviz2 containing one red turtlebot hamburger, a user-defined amount of obstacles, and an arena consisting of four walls. The user has control over the size of the arena, the radius of the cylindrical obstacles, the number of obstacles, and the location of the obstacles. The user can call the /reset service to reset the robot to it's starting position, and they can also call the /teleport service to teleport the robot to a specific xy position and angle.

### Launch File Description:
There is only one launch file in this package, "nusim.launch.xml". This launch file takes the following arguments:
* rate:
    * default="5"
    * description="the period of the timer callback in nusim in milliseconds (ms)"
* config_file:
    * default="basic_world.yaml"
    * description="select the file to configure the simulator with"
* arena_length_x:
    * default="3.0"
    * description="the length of the arena in the x direction"
* arena_length_y:
    * default="3.0"
    * description="the length of the arena in the y direction"
* obstacles/r: 
    * default="0.038"
    * description="the default radius of all column obstacles"

The launch file includes a launch file called load_one.launch.py from nuturtle_description. "nusim.launch.xml" provides this launch file with the following arguments:
* use_rviz: false
* use_jsp: false
* color: red

### Parameters
The following parameters can be used ot change the simulator settings:
* rate: the period of the timer callback in the nusim node in milliseconds (ms)
* arena_length_x: the length of the arena in the x direction
* arena_length_y: the length of the arena in the y direction
* obstacles/r: the radius of the obstacles in the arena

### Image
![Image](/nusim/images/nusim1.png)