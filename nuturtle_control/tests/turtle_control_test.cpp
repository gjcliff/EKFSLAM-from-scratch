#include "rclcpp/rclcpp.hpp"
#include "catch_ros2/catch_ros2.hpp"

using catch_ros2::SimulateArgs;

TEST_CASE("cmd_vel", "[cmd_vel]")
{
  const auto args = SimulateArgs(
    "--ros-args"
    "-p wheel_radius:=0.033"
    "-p track_width:=0.160"
    "-p motor_cmd_max:=265"
    "-p motor_cmd_per_rad_sec:=0.024"
    "-p encoder_ticks_per_rad:=651.898646904"
    "-p collision_radius:=0.11"
  );

  rclcpp::init(args.argc(), args.argv());

  auto node = rclcpp::Node::make_shared("cmd_vel_test_node");

  node->declare_parameter<double>("wheel_radius");
  node->declare_parameter<double>("track_width");
  node->declare_parameter<double>("motor_cmd_max");
  node->declare_parameter<double>("motor_cmd_per_rad_sec");
  node->declare_parameter<double>("encoder_ticks_per_rad");
  node->declare_parameter<double>("collision_radius");

  // rclcpp::Client<geometry_msgs::msg::Twist>::SharedPtr client =
  //   node->create_client<geometry_msgs::msg::Twist

}
