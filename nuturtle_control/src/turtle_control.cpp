/// \file
/// \brief Enable control of the turtlebot via geometry_msgs/msg/Twist messages on the cmd_vel topic
///
/// PARAMETERS:
///     wheel_radius (double): The radius of the turtlebot's wheels
///     track_width (double): The total distance between the center of the turtlebot's wheels
///     motor_cmd_max (double): The max allowable speed of the motor
///     motor_cmd_per_rad_sec (double): The conversion between motor command values and rads/sec
///     encoder_ticks_per_rad (double): The conversion between encoder ticks and rads/sec
///     collision_radius (double): The collision radius of the turtlebot
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
#include <cmath>

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
  : Node("turtle_control"),
    wheel_velocities_({0.0, 0.0})
  {
    // declare parameters
    declare_parameter("wheel_radius", rclcpp::PARAMETER_DOUBLE);
    declare_parameter("track_width", rclcpp::PARAMETER_DOUBLE);
    declare_parameter("motor_cmd_max", rclcpp::PARAMETER_INTEGER);
    declare_parameter("motor_cmd_per_rad_sec", rclcpp::PARAMETER_DOUBLE);
    declare_parameter("encoder_ticks_per_rad", rclcpp::PARAMETER_DOUBLE);
    declare_parameter("collision_radius", rclcpp::PARAMETER_DOUBLE);

    // get parameters

    try {
      wheel_radius_ = get_parameter("wheel_radius").as_double();
      // BEGIN CITATION [24]
    } catch (rclcpp::exceptions::ParameterUninitializedException const &) {
      // END CITATION [24]
      RCLCPP_ERROR_STREAM_ONCE(get_logger(), "no wheel_radius parameter declared");
      // BEGIN CITATION [25]
      throw;
      // END CITATION [25]
    }

    try {
      track_width_ = get_parameter("track_width").as_double();
    } catch (rclcpp::exceptions::ParameterUninitializedException const &) {
      RCLCPP_ERROR_STREAM_ONCE(get_logger(), "no track_width parameter declared");
      throw;
    }

    try {
      motor_cmd_max_ = get_parameter("motor_cmd_max").as_int();
    } catch (rclcpp::exceptions::ParameterUninitializedException const &) {
      RCLCPP_ERROR_STREAM_ONCE(get_logger(), "no motor_cmd_max parameter declared");
      throw;
    }

    try {
      motor_cmd_per_rad_sec_ = get_parameter("motor_cmd_per_rad_sec").as_double();
    } catch (rclcpp::exceptions::ParameterUninitializedException const &) {
      RCLCPP_ERROR_STREAM_ONCE(get_logger(), "no motor_cmd_per_rad_sec parameter declared");
      throw;
    }

    try {
      encoder_ticks_per_rad_ = get_parameter("encoder_ticks_per_rad").as_double();
    } catch (rclcpp::exceptions::ParameterUninitializedException const &) {
      RCLCPP_ERROR_STREAM_ONCE(get_logger(), "no encoder_ticks_per_rad parameter declared");
      throw;
    }

    try {
      collision_radius_ = get_parameter("collision_radius").as_double();
    } catch (rclcpp::exceptions::ParameterUninitializedException const &) {
      RCLCPP_ERROR_STREAM_ONCE(get_logger(), "no collision_radius parameter declared");
      throw;
    }

    turtlelib::RobotDimensions rd{0.0, track_width_ / 2, wheel_radius_};
    turtlebot_.set_robot_dimensions(rd);

    // create subscriber
    cmd_subscriber_ = create_subscription<geometry_msgs::msg::Twist>(
      "cmd_vel", 10, std::bind(&TurtleControl::cmd_callback, this, _1));
    sensor_data_subscriber_ = create_subscription<nuturtlebot_msgs::msg::SensorData>(
      "sensor_data", 10, std::bind(&TurtleControl::sensor_data_callback, this, _1));

    // create publisher
    wheel_cmd_publisher_ = create_publisher<nuturtlebot_msgs::msg::WheelCommands>(
      "wheel_cmd", 10);
    my_joint_state_publisher_ = create_publisher<sensor_msgs::msg::JointState>(
      "/joint_states", 10);

    prev_encoder_tic_time_ = get_clock()->now();

  }

private:
  /// @brief compute a wheel command message converted from a twist
  /// @param msg - the twist to be converted into motor command values
  void cmd_callback(const geometry_msgs::msg::Twist & msg)
  {
    nuturtlebot_msgs::msg::WheelCommands wheel_cmd_msg;
    wheel_velocities_ = turtlebot_.IK({msg.angular.z, msg.linear.x, msg.linear.y});
    wheel_velocities_.at(0) /= motor_cmd_per_rad_sec_;
    wheel_velocities_.at(1) /= motor_cmd_per_rad_sec_;


    for (int i = 0; i < (int)wheel_velocities_.size(); ++i) {
      if (wheel_velocities_.at(i) > motor_cmd_max_) {
        wheel_velocities_.at(i) = motor_cmd_max_;
      } else if (wheel_velocities_.at(i) < -motor_cmd_max_) {
        wheel_velocities_.at(i) = -motor_cmd_max_;
      }
    }
    wheel_cmd_msg.left_velocity = static_cast<int>(wheel_velocities_.at(0));
    wheel_cmd_msg.right_velocity = static_cast<int>(wheel_velocities_.at(1));
    wheel_cmd_publisher_->publish(wheel_cmd_msg);
  }

  /// @brief convert sensor data (encoder ticks) into a joint state for the
  /// turtlebot's two wheels
  /// @param msg - the encoder ticks
  void sensor_data_callback(const nuturtlebot_msgs::msg::SensorData & msg)
  {
    double phi_l = msg.left_encoder / encoder_ticks_per_rad_;
    double phi_r = msg.right_encoder / encoder_ticks_per_rad_;

    double left_wheel_velocity = (phi_l - prev_left_rad_) /
      prev_encoder_tic_time_.seconds();
    double right_wheel_velocity = (phi_r - prev_right_rad_) /
      prev_encoder_tic_time_.seconds();
    prev_left_rad_ = phi_l;
    prev_right_rad_ = phi_r;

    sensor_msgs::msg::JointState joint_state;
    joint_state.header.stamp = msg.stamp;
    joint_state.name = {"wheel_left_joint", "wheel_right_joint"};
    joint_state.position = {phi_l, phi_r};
    joint_state.velocity = {left_wheel_velocity, right_wheel_velocity};

    my_joint_state_publisher_->publish(joint_state);
    prev_encoder_tic_time_ = get_clock()->now();
  }

  rclcpp::Subscription<nuturtlebot_msgs::msg::SensorData>::SharedPtr sensor_data_subscriber_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_subscriber_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr my_joint_state_publisher_;
  rclcpp::Publisher<nuturtlebot_msgs::msg::WheelCommands>::SharedPtr wheel_cmd_publisher_;
  nuturtlebot_msgs::msg::SensorData sensor_data_;
  geometry_msgs::msg::Twist cmd_vel_;
  turtlelib::DiffDrive turtlebot_;

  vector<double> wheel_velocities_;
  rclcpp::Time prev_encoder_tic_time_;
  double prev_left_rad_ = 0.0;
  double prev_right_rad_ = 0.0;
  double wheel_radius_;
  double track_width_;
  int motor_cmd_max_;
  double motor_cmd_per_rad_sec_;
  double encoder_ticks_per_rad_;
  double collision_radius_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TurtleControl>());
  rclcpp::shutdown();
  return 0;
}
