#include <chrono>
#include "catch_ros2/catch_ros2.hpp"
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <string>

using namespace std::chrono_literals;

int count = 0;
void circle_callback(const geometry_msgs::msg::Twist)
{
  count++;
}

TEST_CASE("test_pure_translation", "[cmd_vel_translation]")
{
  auto node = rclcpp::Node::make_shared("turtle_circle_test");

  node->declare_parameter<double>("test_duration");
  node->declare_parameter<int>("frequency");

  const auto TEST_DURATION =
    node->get_parameter("test_duration").get_parameter_value().get<double>();

  auto subscriber = node->create_subscription<geometry_msgs::msg::Twist>(
    "cmd_vel", 10,
    &circle_callback);

  rclcpp::Time start_time = rclcpp::Clock().now();

  while (
    rclcpp::ok() &&
    ((rclcpp::Clock().now() - start_time) < rclcpp::Duration::from_seconds(TEST_DURATION))
  )
  {
    rclcpp::spin_some(node);
  }
  // there seems to be almost exactly a second of delay coming from somewhere,
  // so i set the test_duration to 3 instead of 2. A test duration of 2 results
  // in 200 messages being received, duration of 4 results in 300 messages, and so on.
  CHECK_THAT(count, Catch::Matchers::WithinAbs(static_cast<int>(200), 10));

}
