#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

class TurtleControl : public rclcpp::Node
{
public:
  TurtleControl()
  : Node("TurtleControl"), count_(0)
  {
    // declare parameters
    declare_parameter("wheel_radius", 0.01);
    declare_parameter("track_width", 0.01);
    declare_parameter("motor_cmd_max", 0.01);
    declare_parameter("motor_cmd_per_rad_sec", 0.01);
    declare_parameter("encoder_ticks_per_rad", 0.01);
    declare_parameter("collision_radius", 0.01);

    wheel_radius_ = get_parameter("wheel_radius").as_double();
    track_width_ = get_parameter("track_width").as_double();
    motor_cmd_max_ = get_parameter("motor_cmd_max").as_double();
    motor_cmd_per_rad_sec_ = get_parameter("motor_cmd_per_rad_sec").as_double();
    encoder_ticks_per_rad_ = get_parameter("encoder_ticks_per_rad").as_double();
    collision_radius_ = get_parameter("collision_radius").as_double();

    publisher_ = create_publisher<std_msgs::msg::String>("topic", 10);
    timer_ = create_wall_timer(
      500ms, std::bind(&TurtleControl::timer_callback, this));
  }

private:
  void timer_callback()
  {
    auto message = std_msgs::msg::String();
    message.data = "Hello, world! " + std::to_string(count_++);
    RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
    publisher_->publish(message);
  }
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  double wheel_radius_;
  double track_width_;
  double motor_cmd_max_;
  double motor_cmd_per_rad_sec_;
  double encoder_ticks_per_rad_;
  double collision_radius_;
  size_t count_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TurtleControl>());
  rclcpp::shutdown();
  return 0;
}
