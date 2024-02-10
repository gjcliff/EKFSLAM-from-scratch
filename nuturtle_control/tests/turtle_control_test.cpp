#include <chrono>
#include "catch_ros2/catch_ros2.hpp"
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nuturtlebot_msgs/msg/wheel_commands.hpp"
#include "nuturtlebot_msgs/msg/sensor_data.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include <string>

using namespace std::chrono_literals;

double pt_left_velocity;
double pt_right_velocity;
double pr_left_velocity;
double pr_right_velocity;
bool help = false;
void pure_translation_callback(const nuturtlebot_msgs::msg::WheelCommands & msg)
{
  pt_left_velocity = msg.left_velocity;
  pt_right_velocity = msg.right_velocity;
  help = true;
}

void pure_rotation_callback(const nuturtlebot_msgs::msg::WheelCommands & msg)
{
  pr_left_velocity = msg.left_velocity;
  pr_right_velocity = msg.right_velocity;
}

TEST_CASE("test_pure_translation", "[cmd_vel_translation]")
{
  auto node = rclcpp::Node::make_shared("turtle_control_test");

  node->declare_parameter<double>("test_duration");

  const auto TEST_DURATION =
    node->get_parameter("test_duration").get_parameter_value().get<double>();

  auto cmd_vel_publisher = node->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
  auto wheel_cmd_subscriber = node->create_subscription<nuturtlebot_msgs::msg::WheelCommands>(
    "wheel_cmd", 10, &pure_translation_callback);

  rclcpp::Time start_time = rclcpp::Clock().now();

  bool valid_wheel_cmd = false;

  while (
    rclcpp::ok() &&
    ((rclcpp::Clock().now() - start_time) < rclcpp::Duration::from_seconds(TEST_DURATION))
  )
  {
    geometry_msgs::msg::Twist test_twist;
    test_twist.linear.x = 0.1;
    cmd_vel_publisher->publish(test_twist);
    if (valid_wheel_cmd) {
      break;
    }

    rclcpp::spin_some(node);
  }

  CHECK_THAT(pt_left_velocity, Catch::Matchers::WithinAbs(pt_right_velocity, 1e-5));
}

TEST_CASE("test_pure_rotation", "[cmd_vel_rotation]")
{
  auto node = rclcpp::Node::make_shared("turtle_control_test");

  node->declare_parameter<double>("test_duration");

  const auto TEST_DURATION =
    node->get_parameter("test_duration").get_parameter_value().get<double>();

  auto cmd_vel_publisher = node->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
  auto wheel_cmd_subscriber = node->create_subscription<nuturtlebot_msgs::msg::WheelCommands>(
    "wheel_cmd", 10, &pure_translation_callback);

  rclcpp::Time start_time = rclcpp::Clock().now();

  bool valid_wheel_cmd = false;

  while (
    rclcpp::ok() &&
    ((rclcpp::Clock().now() - start_time) < rclcpp::Duration::from_seconds(TEST_DURATION))
  )
  {
    geometry_msgs::msg::Twist test_twist;
    test_twist.linear.z = 0.1;
    cmd_vel_publisher->publish(test_twist);
    if (valid_wheel_cmd) {
      break;
    }

    rclcpp::spin_some(node);
  }

  CHECK_THAT(pt_left_velocity, Catch::Matchers::WithinAbs(pt_right_velocity, 1e-5));
}
