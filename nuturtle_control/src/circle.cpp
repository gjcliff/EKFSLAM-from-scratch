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
  : Node("minimal_publisher"), count_(0)
  {
    declare_parameter("frequency_", "100");
    frequency_ = get_parameter("frequency").as_double();
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
  double calculate_vx(double r)
  {
    // I have math to back this up
    return turtlelib::PI * r;
  }
  void control_callback(
    const std::shared_ptr<nuturtle_control::srv::Control::Request> request,
    std::shared_ptr<nuturtle_control::srv::Control::Response>)
  {
    // I have math to back the line below up
    double vx = turtlelib::PI * request->radius;
    geometry_msgs::msg::Twist msg;
    cmd_vel_msg_.angular.z = request->velocity;
    cmd_vel_msg_.linear.x = vx;


  }
  void reverse_callback(
    const std::shared_ptr<std_srvs::srv::Empty::Request>,
    std::shared_ptr<std_srvs::srv::Empty::Response>)
  {
    cmd_vel_msg_.angular.z *= -1.0;
    cmd_vel_msg_.linear.x *= -1.0;
  }
  void stop_callback(
    const std::shared_ptr<std_srvs::srv::Empty::Request>,
    std::shared_ptr<std_srvs::srv::Empty::Response>)
  {
    cmd_vel_msg_.angular.z = 0.0;
    cmd_vel_msg_.linear.x = 0.0;
  }
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
