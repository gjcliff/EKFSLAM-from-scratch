/// \file
/// \brief A brief description of what the file does
///
/// PARAMETERS:
///     frequency (int): The frequency at which circle publishes cmd_vel messages
/// PUBLISHES:
///     cmd_vel (geometry_msgs::msg::Twist): Publish twists for the turtlebot to follow
/// SERVERS:
///     control (nuturtle_control::srv::Control): Command the robot to move in a circle,
///     specifying an angular velocity and a radius
///     reverse (std_srvs::srv::Empty): Command the robot to reverse it's current trajectory
///     stop (std_srvs::srv::Empty): Command the robot to stop

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nuturtle_control/srv/control.hpp"
#include "std_srvs/srv/empty.hpp"
#include "turtlelib/geometry2d.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1, std::placeholders::_2;

class Circle : public rclcpp::Node
{
public:
  Circle()
  : Node("circle"), count_(0)
  {
    declare_parameter("frequency", 100);
    frequency_ = get_parameter("frequency").as_int();
    std::chrono::duration<double> period(1.0 / frequency_);

    // create publishers
    cmd_vel_publisher_ = create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

    // create services
    control_service_ =
      create_service<nuturtle_control::srv::Control>(
      "control",
      std::bind(&Circle::control_callback, this, _1, _2));
    reverse_service_ =
      create_service<std_srvs::srv::Empty>(
      "reverse",
      std::bind(&Circle::reverse_callback, this, _1, _2));
    stop_service_ =
      create_service<std_srvs::srv::Empty>("stop", std::bind(&Circle::stop_callback, this, _1, _2));

    // create timer
    timer_ = create_wall_timer(
      period, std::bind(&Circle::timer_callback, this));

    // initialize variables
    cmd_vel_msg_.angular.z = 0.0;
    cmd_vel_msg_.linear.x = 0.0;
  }

private:
  /// @brief convert an angular velocity and a radius of a circle into a twist
  /// @param request - an angular velocity and a radius
  /// @param - an empty response
  void control_callback(
    const std::shared_ptr<nuturtle_control::srv::Control::Request> request,
    std::shared_ptr<nuturtle_control::srv::Control::Response>)
  {
    geometry_msgs::msg::Twist msg;
    if (request->radius == 0.0) {
      cmd_vel_msg_.angular.z = 0.0;
      cmd_vel_msg_.linear.x = request->velocity;
    } else {
      cmd_vel_msg_.angular.z = request->velocity;
      cmd_vel_msg_.linear.x = request->velocity * request->radius;
    }
  }

  /// @brief command the robot to reverse its trajectory
  /// @param - an empty request
  /// @param - an empty response
  void reverse_callback(
    const std::shared_ptr<std_srvs::srv::Empty::Request>,
    std::shared_ptr<std_srvs::srv::Empty::Response>)
  {
    cmd_vel_msg_.angular.z *= -1.0;
    cmd_vel_msg_.linear.x *= -1.0;
  }

  /// @brief command the robot to stop
  /// @param - an empty request
  /// @param - an empty response
  void stop_callback(
    const std::shared_ptr<std_srvs::srv::Empty::Request>,
    std::shared_ptr<std_srvs::srv::Empty::Response>)
  {
    cmd_vel_msg_.angular.z = 0.0;
    cmd_vel_msg_.linear.x = 0.0;
  }

  /// @brief constantly publish a twist for the robot to follow, sometimes
  /// this twist tells the robot to do nothing
  void timer_callback()
  {
    cmd_vel_publisher_->publish(cmd_vel_msg_);
  }
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;
  rclcpp::Service<nuturtle_control::srv::Control>::SharedPtr control_service_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr reverse_service_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr stop_service_;
  rclcpp::TimerBase::SharedPtr timer_;
  geometry_msgs::msg::Twist cmd_vel_msg_;
  int frequency_;
  size_t count_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Circle>());
  rclcpp::shutdown();
  return 0;
}
