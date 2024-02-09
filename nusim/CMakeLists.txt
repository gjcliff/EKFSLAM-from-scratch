cmake_minimum_required(VERSION 3.8)
project(nusim)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

if(NOT CMAKE_BUILD_TYPE)        # If no build type is set, set one
  set(CMAKE_BUILD_TYPE "Debug" CACHE STRING "Type of build." FORCE)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(turtlelib)
find_package(nuturtlebot_msgs)

set(srv_files
  "srv/Teleport.srv"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${srv_files}
)


ament_export_dependencies(rosidl_default_runtime)

add_executable(nusim_exec src/nusim.cpp)

target_link_libraries(nusim_exec turtlelib::turtlelib)

ament_target_dependencies(
  nusim_exec
  std_msgs
  std_srvs
  geometry_msgs
  visualization_msgs
  rclcpp
  tf2
  tf2_ros
  tf2_geometry_msgs
  turtlelib
  nuturtlebot_msgs
)

install(TARGETS
  nusim_exec
  DESTINATION lib/${PROJECT_NAME}
)

# install launch files
install(
  DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# install config directory
install(
  DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} rosidl_typesupport_cpp)

target_link_libraries(nusim_exec "${cpp_typesupport_target}")

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
