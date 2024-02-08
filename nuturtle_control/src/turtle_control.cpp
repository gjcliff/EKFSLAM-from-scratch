/// \file
/// \brief Enable control of the turtlebot via geometry_msgs/msg/Twist messages on the cmd_vel topic
///
/// PARAMETERS:
///     wheel_radius (double): The radius of the turtlebot's wheels
///     track_width (double): The total distance between the center of the turtlebot's wheels
///     motor_cmd_max (double): The max allowable speed of the motor
///     motor_cmd_per_rad_sec (double):
///     encoder_ticks_per_rad (double):
///     collision_radius (double):
/// PUBLISHES:
///     wheel_cmd (nuturtlebot_msgs::msg::WheelCOmmands): Commands that make the turtlebot follow a specified twist
///     joint_states (sensor_msgs::msg::JointState): Provide the angle (rad) and velocity (rad/sec) of the turtlebot's wheels
/// SUBSCRIBES:
///     cmd_vel (geometry_msgs::msg::Twist): A twist commanded to the turtlebot
///     sensor_data (nuturtlebot_msgs::msg::SensorData): A stream of data from the turtlebot including
///     encoder data, accelerometer data, gyroscope data, magnetometer data, and batter information.

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

#include "turtlelib/diff_drive.hpp"
#include "turtlelib/geometry2d.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

class TurtleControl : public rclcpp::Node
{
public:
  TurtleControl()
  : Node("TurtleControl"),
    wheel_velocities_({0.0, 0.0}),
    count_(0)
  {
    // declare parameters
    declare_parameter("wheel_radius", 0.01);
    declare_parameter("track_width", 0.01);
    declare_parameter("motor_cmd_max", 0.01);
    declare_parameter("motor_cmd_per_rad_sec", 0.01);
    declare_parameter("encoder_ticks_per_rad", 0.01);
    declare_parameter("collision_radius", 0.01);

    // get parameters
    try {
      wheel_radius_ = get_parameter("wheel_radius").as_double();
    } catch (rclcpp::exceptions::ParameterUninitializedException) {
      rclcpp::shutdown();
    }
    try {
      track_width_ = get_parameter("track_width").as_double();
    } catch (rclcpp::exceptions::ParameterUninitializedException) {
      rclcpp::shutdown();
    }
    try {
      motor_cmd_max_ = get_parameter("motor_cmd_max").as_double();
    } catch (rclcpp::exceptions::ParameterUninitializedException) {
      rclcpp::shutdown();
    }
    try {
      motor_cmd_per_rad_sec_ = get_parameter("motor_cmd_per_rad_sec").as_double();
    } catch (rclcpp::exceptions::ParameterUninitializedException) {
      rclcpp::shutdown();
    }
    try {
      encoder_ticks_per_rad_ = get_parameter("encoder_ticks_per_rad").as_double();
    } catch (rclcpp::exceptions::ParameterUninitializedException) {
      rclcpp::shutdown();
    }
    try {
      collision_radius_ = get_parameter("collision_radius").as_double();
    } catch (rclcpp::exceptions::ParameterUninitializedException) {
      rclcpp::shutdown();
    }

    // create subscriber
    cmd_subscriber_ = create_subscription<geometry_msgs::msg::Twist>(
      "cmd_vel", 10, std::bind(&TurtleControl::cmd_callback, this, _1));
    sensor_data_subscriber_ = create_subscription<nuturtlebot_msgs::msg::SensorData>(
      "sensor_data", 10, std::bind(&TurtleControl::sensor_data_callback, this, _1));

    // create publisher
    wheel_cmd_publisher_ = create_publisher<nuturtlebot_msgs::msg::WheelCommands>("wheel_cmd", 10);
    my_joint_state_publisher_ = create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);

    // create timer
    timer_ = create_wall_timer(
      500ms, std::bind(&TurtleControl::timer_callback, this));

    prev_encoder_tic_time_ = get_clock()->now();
  }

private:
  void sensor_data_callback(const nuturtlebot_msgs::msg::SensorData & msg)
  {
    RCLCPP_INFO_STREAM_ONCE(get_logger(), "got sensor_data msg");

    double phi_l = get_encoder_angle(msg.left_encoder);
    double phi_r = get_encoder_angle(msg.right_encoder);

    double left_wheel_velocity = (msg.left_encoder - prev_left_encoder_)/(prev_encoder_tic_time_.nanoseconds()/1e9);
    double right_wheel_velocity = (msg.right_encoder - prev_right_encoder_)/(prev_encoder_tic_time_.nanoseconds()/1e9);
    prev_left_encoder_ = msg.left_encoder;
    prev_right_encoder_ = msg.right_encoder;

    sensor_msgs::msg::JointState joint_state;
    joint_state.position = {phi_l, phi_r};
    joint_state.velocity = {left_wheel_velocity, right_wheel_velocity};

    my_joint_state_publisher_->publish(joint_state);
  }

  void cmd_callback(const geometry_msgs::msg::Twist & msg)
  {
    RCLCPP_INFO_STREAM_ONCE(get_logger(), "got cmd_vel msg");

    nuturtlebot_msgs::msg::WheelCommands wheel_cmd_msg;
    wheel_velocities_ = turtlebot_.IK({msg.angular.z, msg.linear.x, msg.linear.y});
    wheel_velocities_.at(0) *= motor_cmd_per_rad_sec_;
    wheel_velocities_.at(1) *= motor_cmd_per_rad_sec_;

    for (int i = 0; i < (int)wheel_velocities_.size(); ++i){
      if (wheel_velocities_.at(i) > motor_cmd_max_) {
        wheel_velocities_.at(i) = motor_cmd_max_;
      } else if (wheel_velocities_.at(i) < -motor_cmd_max_) {
        wheel_velocities_.at(i) = -motor_cmd_max_;
      }
    }

    wheel_cmd_msg.left_velocity = wheel_velocities_.at(0);
    wheel_cmd_msg.right_velocity = wheel_velocities_.at(1);

    wheel_cmd_publisher_->publish(wheel_cmd_msg);
  }

  float get_encoder_angle(int encoder_ticks)
  {
    return turtlelib::normalize_angle(encoder_ticks * encoder_ticks_per_rad_);
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
  nuturtlebot_msgs::msg::SensorData sensor_data_;
  geometry_msgs::msg::Twist cmd_vel_;
  turtlelib::DiffDrive turtlebot_;
  vector<double> wheel_velocities_;
  rclcpp::Time prev_encoder_tic_time_;
  int prev_left_encoder_;
  int prev_right_encoder_;
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
