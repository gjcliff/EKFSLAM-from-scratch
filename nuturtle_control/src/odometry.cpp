#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "nav_msgs/msg/odometry.hpp"

#include "turtlelib/geometry2d.hpp"
#include "turtlelib/diff_drive.hpp"
#include <string>
#include <vector>

using namespace std::chrono_literals;
using std::placeholders::_1;

/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */

class Odometry : public rclcpp::Node
{
public:
  Odometry()
  : Node("odometry"), count_(0)
  {
    declare_parameter("body_id", "not_specified");
    declare_parameter("odom_id", "odom");
    declare_parameter("wheel_left", "not_specified");
    declare_parameter("wheel_right", "not_specified");

    try {
      body_id_ = get_parameter("body_id").as_string();
    } catch (rclcpp::exceptions::ParameterUninitializedException) {
      rclcpp::shutdown();
    }

    try {
      odom_id_ = get_parameter("odom_id").as_string();
    } catch (rclcpp::exceptions::ParameterUninitializedException) {
      rclcpp::shutdown();
    }

    try {
      wheel_left_ = get_parameter("wheel_left").as_string();
    } catch (rclcpp::exceptions::ParameterUninitializedException) {
      rclcpp::shutdown();
    }

    try {
      wheel_right_ = get_parameter("wheel_right").as_string();
    } catch (rclcpp::exceptions::ParameterUninitializedException) {
      rclcpp::shutdown();
    }

    // create publishers
    odometry_publisher_ = create_publisher<nav_msgs::msg::Odometry>(
      "odom", 10);

    // create subscribers
    joint_state_subscriber_ = create_subscription<sensor_msgs::msg::JointState>(
      "joint_states", 10, std::bind(&Odometry::joint_state_callback, this, _1));

    timer_ = this->create_wall_timer(
      500ms, std::bind(&Odometry::timer_callback, this));
  }

private:
  void joint_state_callback(const sensor_msgs::msg::JointState msg)
  {
    double phi_l = msg.position.at(0);
    double phi_r = msg.position.at(1);

    turtlelib::Twist2D Vb = turtlebot_.FK(phi_l, phi_r);
    turtlelib::Configuration q_diff = turtlebot_.update_configuration(Vb);

    nav_msgs::msg::Odometry odom_msg;
    odom_msg.header.frame_id = body_id_;
    odom_msg.child_frame_id = odom_id_;
    // odom_msg.pose
    odom_msg.twist.twist.angular.z = Vb.omega;
    odom_msg.twist.twist.linear.x = Vb.x;
    odom_msg.twist.twist.linear.y = Vb.y;

  }
  void timer_callback()
  {

  }
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odometry_publisher_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_subscriber_;
  turtlelib::DiffDrive turtlebot_;
  std::string body_id_;
  std::string odom_id_;
  std::string wheel_left_;
  std::string wheel_right_;
  size_t count_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Odometry>());
  rclcpp::shutdown();
  return 0;
}
