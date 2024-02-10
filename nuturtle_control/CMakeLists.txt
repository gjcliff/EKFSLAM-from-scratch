cmake_minimum_required(VERSION 3.8)
project(nuturtle_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

if(NOT CMAKE_BUILD_TYPE)        # If no build type is set, set one
  set(CMAKE_BUILD_TYPE "Debug" CACHE STRING "Type of build." FORCE)
endif()

find_package(Doxygen)
# Building documentation should be optional.
# To build documentation pass -DBUILD_DOCS=ON when generating the build system
option(BUILD_DOCS "Build the documentation" OFF)

# build just because Doxygen is missing
if(${DOXYGEN_FOUND} AND ${BUILD_DOCS})
    # Turn the README.md into the homepage of the doxygen docs
    set(DOXYGEN_USE_MDFILE_AS_MAINPAGE README.md)

    # Tell Doxygen where to find the documentation
    doxygen_add_docs(doxygen include/ src/ README.md ALL)

    # The documentation will be in the build/html directory
    # The main page is build/html/index.html
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(nuturtle_description REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nuturtlebot_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(numsr_turtlebot REQUIRED)


# find libraries
find_package(turtlelib REQUIRED)

# find test pkg


add_executable(turtle_control src/turtle_control.cpp)
add_executable(odometry src/odometry.cpp)
add_executable(circle src/circle.cpp)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/RobotConfiguration.msg"
  "srv/InitialPose.srv"
  "srv/Control.srv"
)

ament_target_dependencies(
  turtle_control
  rclcpp
  nav_msgs
  sensor_msgs
  turtlelib
  nuturtle_description
  nuturtlebot_msgs
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  std_srvs
)

ament_target_dependencies(
  odometry
  rclcpp
  nav_msgs
  sensor_msgs
  turtlelib
  nuturtle_description
  nuturtlebot_msgs
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  std_srvs
)

ament_target_dependencies(
  circle
  rclcpp
  nav_msgs
  sensor_msgs
  turtlelib
  nuturtle_description
  nuturtlebot_msgs
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  std_srvs
)

rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} rosidl_typesupport_cpp)

target_link_libraries(odometry "${cpp_typesupport_target}")
target_link_libraries(circle "${cpp_typesupport_target}")
target_link_libraries(turtle_control "${cpp_typesupport_target}")
target_link_libraries(odometry turtlelib::turtlelib)
target_link_libraries(turtle_control turtlelib::turtlelib)
target_link_libraries(circle turtlelib::turtlelib)

target_include_directories(turtle_control PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(turtle_control PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

install(TARGETS
  turtle_control
  odometry
  circle
  DESTINATION lib/${PROJECT_NAME})

# install launch files
install(
  DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

install(
  DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

if(BUILD_TESTING)
  # find testing dependencies
  find_package(catch_ros2 REQUIRED)

  include(CTest)

  add_executable(turtle_control_test tests/turtle_control_test.cpp)
  add_executable(turtle_odom_test tests/turtle_odom_test.cpp)
  add_executable(turtle_circle_test tests/turtle_circle_test.cpp)

  target_link_libraries(turtle_control_test
  catch_ros2::catch_ros2_with_node_main)

  target_link_libraries(turtle_odom_test
  catch_ros2::catch_ros2_with_node_main
  "${cpp_typesupport_target}")

  target_link_libraries(turtle_circle_test
  catch_ros2::catch_ros2_with_node_main)

  ament_target_dependencies(turtle_control_test
    rclcpp
    geometry_msgs
    nuturtlebot_msgs
    sensor_msgs
  )

  ament_target_dependencies(turtle_odom_test
    rclcpp
    tf2_ros
  )

  ament_target_dependencies(turtle_circle_test
    rclcpp
    geometry_msgs
  )

  install(TARGETS
    turtle_control_test
    turtle_odom_test
    turtle_circle_test
    DESTINATION lib/${PROJECT_NAME}
  )

  install(FILES
    tests/turtle_control_test.launch.xml
    tests/turtle_odom_test.launch.xml
    tests/turtle_circle_test.launch.xml
    DESTINATION share/${PROJECT_NAME}
  )

  catch_ros2_add_integration_test(TurtleControlIntegration_TestCpp
    LAUNCH_FILE turtle_control_test.launch.xml
  )

  catch_ros2_add_integration_test(TurtleOdomIntegration_TestCpp
    LAUNCH_FILE turtle_odom_test.launch.xml
  )

  catch_ros2_add_integration_test(TurtleCircleIntegration_TestCpp
    LAUNCH_FILE turtle_circle_test.launch.xml
  )

  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
