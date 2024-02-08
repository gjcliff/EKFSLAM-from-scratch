cmake_minimum_required(VERSION 3.8)
project(nuturtle_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

if(NOT CMAKE_BUILD_TYPE)        # If no build type is set, set one
  set(CMAKE_BUILD_TYPE "Debug" CACHE STRING "Type of build." FORCE)
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

add_executable(turtle_control src/turtle_control.cpp)
add_executable(odometry src/odometry.cpp)
add_executable(circle src/circle.cpp)

# find testing dependencies
find_package(catch_ros2 REQUIRED)

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
  include(CTest)

  add_executable(turtle_control_test tests/turtle_control_test.cpp)
  target_link_libraries(turtle_control_test
  catch_ros2::catch_ros2_with_main)

  add_test(NAME cmd_vel_test COMMAND turtle_control_test)

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
