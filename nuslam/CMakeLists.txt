cmake_minimum_required(VERSION 3.8)
project(nuslam)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(turtlelib REQUIRED)
find_package(Armadillo REQUIRED)
find_package(rosidl_default_generators REQUIRED)


include_directories(${ARMADILLO_INCLUDE_DIRS})

add_executable(slam src/slam.cpp)
add_executable(landmarks src/landmarks.cpp)


ament_target_dependencies(
    slam 
    rclcpp
    visualization_msgs
    nav_msgs
    tf2_ros
    tf2
    tf2_geometry_msgs
    turtlelib
    Armadillo
)

ament_target_dependencies(
    landmarks
    rclcpp
    sensor_msgs
    tf2_ros
    visualization_msgs
    turtlelib
    Armadillo
)

# begin custom mesage stuff

set(msg_files
    "msg/Circle.msg"
    "msg/Landmarks.msg"
)

rosidl_generate_interfaces(${PROJECT_NAME}
    ${msg_files}
)

rosidl_get_typesupport_target(cpp_typesupport_target
    ${PROJECT_NAME} rosidl_typesupport_cpp)

target_link_libraries(slam "${cpp_typesupport_target}" ${ARMADILLO_LIBRARIES} turtlelib::turtlelib)
target_link_libraries(landmarks "${cpp_typesupport_target}" ${ARMADILLO_LIBRARIES} turtlelib::turtlelib)

ament_export_dependencies(rosidl_default_runtime)

# end custom message stuff

install(TARGETS
  slam
  landmarks
  DESTINATION lib/${PROJECT_NAME}
)

# install launch files
install(
  DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# # install config directory
# install(
#   DIRECTORY config/
#   DESTINATION share/${PROJECT_NAME}/config
# )

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
