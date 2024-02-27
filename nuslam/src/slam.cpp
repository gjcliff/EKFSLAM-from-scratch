#include <chrono>
#include <string>
#include <vector>
#include <cmath>
#include <random>
#include <armadillo>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "turtlelib/diff_drive.hpp"
#include "turtlelib/geometry2d.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

class Slam : public rclcpp::Node
{
  public:
    Slam()
    : Node("slam"), count_(0)
    {
      declare_parameter("wheel_radius", rclcpp::PARAMETER_DOUBLE);
      declare_parameter("track_width", rclcpp::PARAMETER_DOUBLE);
      declare_parameter("motor_cmd_max", rclcpp::PARAMETER_INTEGER);
      declare_parameter("motor_cmd_per_rad_sec", rclcpp::PARAMETER_DOUBLE);
      declare_parameter("encoder_ticks_per_rad", rclcpp::PARAMETER_DOUBLE);
      declare_parameter("collision_radius", rclcpp::PARAMETER_DOUBLE);

      try {
        body_id_ = get_parameter("body_id").as_string();
      } catch (rclcpp::exceptions::InvalidParameterTypeException()) {
        RCLCPP_ERROR_STREAM(get_logger(), "no body_id parameter declared");
      }

      try {
        odom_id_ = get_parameter("odom_id").as_string();
      } catch (rclcpp::exceptions::InvalidParameterTypeException()) {
        RCLCPP_ERROR_STREAM(get_logger(), "no odom_id parameter declared");
      }

      try {
        wheel_left_ = get_parameter("wheel_left").as_string();
      } catch (rclcpp::exceptions::InvalidParameterTypeException()) {
        RCLCPP_ERROR_STREAM(get_logger(), "no wheel_left parameter declared");
      }

      try {
        wheel_right_ = get_parameter("wheel_right").as_string();
      } catch (rclcpp::exceptions::InvalidParameterTypeException()) {
        RCLCPP_ERROR_STREAM(get_logger(), "no wheel_right parameter declared");
      }

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

      // set the dimensions for the turtlebot
      turtlelib::RobotDimensions rd{0.0, track_width_ / 2, wheel_radius_};
      turtlebot_.set_robot_dimensions(rd);

      joint_state_subscription_ = create_subscription<sensor_msgs::msg::JointState>(
        "joint_states", 10, std::bind(&Slam::joint_state_callback, this, _1));

      timer_ = this->create_wall_timer(
      200ms, std::bind(&Slam::timer_callback, this));
    }

  private:
    void joint_state_callback(const sensor_msgs::msg::JointState & msg)
    {
      double phi_l = msg.position.at(0);
      double phi_r = msg.position.at(1);

      double phi_delta_l = phi_l - phi_l_prev_;
      double phi_delta_r = phi_r - phi_r_prev_;

      phi_l_prev_ = phi_l;
      phi_r_prev_ = phi_r;

      turtlelib::Twist2D Vb = turtlebot_.FK(phi_delta_l, phi_delta_r);

    }
    void timer_callback()
    {
      
    }
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_subscription_;

    turtlelib::DiffDrive turtlebot_;
    std::string body_id_;
    std::string odom_id_;
    std::string wheel_left_;
    std::string wheel_right_;
    double wheel_radius_;
    double track_width_;
    int motor_cmd_max_;
    double motor_cmd_per_rad_sec_;
    double encoder_ticks_per_rad_;
    double collision_radius_;
    double phi_l_prev_ = 0.0;
    double phi_r_prev_ = 0.0;

    size_t count_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Slam>());
  rclcpp::shutdown();
  return 0;
}
