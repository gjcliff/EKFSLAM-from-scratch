#include <chrono>
#include "catch_ros2/catch_ros2.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "nuturtle_control/srv/initial_pose.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include <string>

using namespace std::chrono_literals;
bool transform_found = false;

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

TEST_CASE("test_transform_listener", "[transform_listener]")
{
  auto node = rclcpp::Node::make_shared("turtle_odom_test");

  node->declare_parameter<double>("test_duration");

  const auto TEST_DURATION =
    node->get_parameter("test_duration").get_parameter_value().get<double>();

  auto tf_buffer = std::make_unique<tf2_ros::Buffer>(node->get_clock());
  auto transform_listener = tf2_ros::TransformListener{*tf_buffer};

  rclcpp::Time start_time = rclcpp::Clock().now();

  while (
    rclcpp::ok() &&
    ((rclcpp::Clock().now() - start_time) < rclcpp::Duration::from_seconds(TEST_DURATION))
  )
  {
    try {
      geometry_msgs::msg::TransformStamped t;
      t = tf_buffer->lookupTransform("base_footprint", "odom", tf2::TimePointZero);
      transform_found = true;
      break;
    } catch (const tf2::TransformException & ex) {
      break;
    }

  }

  CHECK(transform_found);

}
