cmake_minimum_required(VERSION 3.8)
project(nuturtle_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(nuturtle_description REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nuturtlebot_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

# find libraries
find_package(turtlelib REQUIRED)



add_executable(turtle_control src/turtle_control.cpp)
add_executable(odometry src/odometry.cpp)
add_executable(circle src/circle.cpp)

ament_target_dependencies(
  turtle_control
  rclcpp
  nav_msgs
  sensor_msgs
  turtlelib
  nuturtle_description
  nuturtlebot_msgs
  geometry_msgs
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
)

target_link_libraries(turtle_control turtlelib::turtlelib)
target_link_libraries(odometry turtlelib::turtlelib)
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

if(BUILD_TESTING)
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
