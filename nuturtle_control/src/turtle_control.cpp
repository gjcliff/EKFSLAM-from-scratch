#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "nuturtlebot_msgs/msg/wheel_commands.hpp"
#include "nuturtlebot_msgs/msg/sensor_data.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

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

    // get parameters
    wheel_radius_ = get_parameter("wheel_radius").as_double();
    track_width_ = get_parameter("track_width").as_double();
    motor_cmd_max_ = get_parameter("motor_cmd_max").as_double();
    motor_cmd_per_rad_sec_ = get_parameter("motor_cmd_per_rad_sec").as_double();
    encoder_ticks_per_rad_ = get_parameter("encoder_ticks_per_rad").as_double();
    collision_radius_ = get_parameter("collision_radius").as_double();

    // create subscriber
    cmd_subscriber_ = create_subscription<geometry_msgs::msg::Twist>(
      "cmd_vel", 10, std::bind(&TurtleControl::cmd_callback, this, _1));
    sensor_data_subscriber_ = create_subscription<nuturtlebot_msgs::msg::SensorData>(
      "sensor_data", 10, std::bind(&TurtleControl::sensor_data_callback, this, _1));

    // create publisher
    wheel_cmd_publisher_ = create_publisher<nuturtlebot_msgs::msg::WheelCommands>("topic", 10);
    my_joint_state_publisher_ = create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);

    // create timer
    timer_ = create_wall_timer(
      500ms, std::bind(&TurtleControl::timer_callback, this));
  }

private:
  void sensor_data_callback(const nuturtlebot_msgs::msg::SensorData & msg) const
  {
    RCLCPP_INFO_STREAM_ONCE(get_logger(), "got sensor_data msg");
  }

  void cmd_callback(const geometry_msgs::msg::Twist & msg) const
  {
    RCLCPP_INFO_STREAM_ONCE(get_logger(), "got cmd_vel msg");
  }

  void timer_callback()
  {
    nuturtlebot_msgs::msg::WheelCommands msg;
    wheel_cmd_publisher_->publish(msg);
  }
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Subscription<nuturtlebot_msgs::msg::SensorData>::SharedPtr sensor_data_subscriber_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_subscriber_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr my_joint_state_publisher_;
  rclcpp::Publisher<nuturtlebot_msgs::msg::WheelCommands>::SharedPtr wheel_cmd_publisher_;
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
