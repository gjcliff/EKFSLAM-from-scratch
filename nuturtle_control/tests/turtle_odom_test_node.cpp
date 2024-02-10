#include <chrono>
#include "catch_ros2/catch_ros2.hpp"
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nuturtlebot_msgs/msg/wheel_commands.hpp"
#include "nuturtlebot_msgs/msg/sensor_data.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "nuturtle_control/srv/initial_pose.hpp"
#include <string>

using namespace std::chrono_literals;

TEST_CASE("test_initial_pose", "[initial_pose]")
{
  auto node = rclcpp::Node::make_shared("turtle_odom_test");

  node->declare_parameter<double>("test_duration");

  const auto TEST_DURATION =
    node->get_parameter("test_duration").get_parameter_value().get<double>();

  auto initial_pose_client =
    node->create_client<nuturtle_control::srv::InitialPose>("initial_pose");

  rclcpp::Time start_time = rclcpp::Clock().now();
  bool initial_pose_service_found = false;

  while (
    rclcpp::ok() &&
    ((rclcpp::Clock().now() - start_time) < rclcpp::Duration::from_seconds(TEST_DURATION))
  )
  {
    if (initial_pose_client->wait_for_service(0s)) {
      initial_pose_service_found = true;
      break;
    }
  }

  CHECK(initial_pose_service_found);
}
