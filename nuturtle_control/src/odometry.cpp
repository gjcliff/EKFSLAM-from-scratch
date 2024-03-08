/// \file
/// \brief perform odometry calculations for the turtlebot
///
/// PARAMETERS:
///     body_id (string): The name of the turtlebot's base_footprint frame
///     odom_id (string): The name of the odom frame
///     wheel_left (string): The name of the left wheel's joint
///     wheel_right (string): The name of the right wheel's joint
///     wheel_radius (double): The radius of the turtlebot's wheels
///     track_width (double): The total distance between the center of the turtlebot's wheels
///     motor_cmd_max (double): The max allowable speed of the motor
///     motor_cmd_per_rad_sec (double): The conversion between motor command values and rads/sec
///     encoder_ticks_per_rad (double): The conversion between encoder ticks and rads/sec
///     collision_radius (double): The collision radius of the turtlebot
/// PUBLISHES:
///     odom (nav_msgs::msg::Odometry): Publish odometry messages for the turtlebot
/// SUBSCRIBES:
///     joint_states (sensor_msgs::msg::JointState): Subscribe to joint state messages for the
///     turtlebot's wheels
/// SERVERS:
///     initial_pose (nuturtle_control::srv::InitialPose): Reset the location of the odometry so
///     that the robot thinks it's at the requested location
/// BROADCASTER:
///     transform_broadcaster (geometry_msgs::msg::TransformStamped): Broadcast the location of the
///     base link of the turtlebot in relation to the odom frame

#include <chrono>
#include <functional>
#include <geometry_msgs/msg/detail/pose_stamped__struct.hpp>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_ros/transform_broadcaster.h"

#include "nuturtle_control/srv/initial_pose.hpp"
#include "nuturtle_control/msg/robot_configuration.hpp"

#include "turtlelib/geometry2d.hpp"
#include "turtlelib/diff_drive.hpp"

#include <string>

using namespace std::chrono_literals;
using std::placeholders::_1, std::placeholders::_2;

class Odometry : public rclcpp::Node
{
public:
  Odometry()
  : Node("odometry")
  {
    declare_parameter("body_id", rclcpp::PARAMETER_STRING);
    declare_parameter("odom_id", rclcpp::PARAMETER_STRING);
    declare_parameter("wheel_left", rclcpp::PARAMETER_STRING);
    declare_parameter("wheel_right", rclcpp::PARAMETER_STRING);

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

    turtlelib::RobotDimensions rd{0.0, track_width_ / 2, wheel_radius_};
    turtlebot_.set_robot_dimensions(rd);

    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    // create publishers
    odometry_publisher_ = create_publisher<nav_msgs::msg::Odometry>(
      "odom", 10);

    path_publisher_ = create_publisher<nav_msgs::msg::Path>(
      "path", 10);

    // create subscribers
    joint_state_subscriber_ = create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", 10, std::bind(&Odometry::joint_state_callback, this, _1));

    // create services
    initial_pose_service_ = create_service<nuturtle_control::srv::InitialPose>(
      "initial_pose", std::bind(&Odometry::initial_pose_callback, this, _1, _2));

    // timer_ = this->create_wall_timer(
    //   10ms, std::bind(&Odometry::timer_callback, this));

    robot_odometry_.header.frame_id = odom_id_;
    robot_odometry_.child_frame_id = body_id_;

    path_.header.frame_id = "nusim/world";
  }

private:
  geometry_msgs::msg::PoseStamped construct_path_msg()
  {
    geometry_msgs::msg::PoseStamped pose;
    pose.header.stamp = get_clock()->now();
    pose.header.frame_id = "nusim/world";
    pose.pose.position.x = x_;
    pose.pose.position.y = y_;

    tf2::Quaternion q_tf2;
    q_tf2.setRPY(0, 0, theta_);
    pose.pose.orientation = tf2::toMsg(q_tf2);

    return pose;
  }
  /// @brief reset the location of the odometry calculations
  /// @param request - the requested new location for odometry calculations
  /// @param - an empty response
  void initial_pose_callback(
    const std::shared_ptr<nuturtle_control::srv::InitialPose::Request> request,
    std::shared_ptr<nuturtle_control::srv::InitialPose::Response>)
  {
    tf2::Quaternion tf2_quat;
    tf2_quat.setRPY(0.0, 0.0, request->q.theta);

    robot_odometry_.pose.pose.orientation = tf2::toMsg(tf2_quat);
    robot_odometry_.pose.pose.position.x = request->q.x;
    robot_odometry_.pose.pose.position.y = request->q.y;

    x_ = request->q.x;
    y_ = request->q.y;
    theta_ = request->q.theta;
  }

  /// @brief perform FK on the joint positions of the turtlebot's wheels
  /// in radians and update the location of the turtlebot's base frame
  /// @param msg - the joint positions of the turtlebot's wheels
  void joint_state_callback(const sensor_msgs::msg::JointState & msg)
  {
    const double phi_l = msg.position.at(0);
    const double phi_r = msg.position.at(1);

    const double phi_delta_l = phi_l - phi_l_prev_;
    const double phi_delta_r = phi_r - phi_r_prev_;

    phi_l_prev_ = phi_l;
    phi_r_prev_ = phi_r;

    turtlelib::Twist2D Vb = turtlebot_.FK(phi_delta_l, phi_delta_r);
    turtlelib::Configuration q_now = turtlebot_.update_configuration(Vb);

    x_ = q_now.x;
    y_ = q_now.y;
    theta_ = q_now.theta;

    tf2::Quaternion tf2_quat;
    tf2_quat.setRPY(0.0, 0.0, q_now.theta);

    // BEGIN CITATION [23]
    geometry_msgs::msg::Quaternion geometry_quat = tf2::toMsg(tf2_quat);
    // END CITATION [23]

    robot_odometry_.header.frame_id = odom_id_;
    robot_odometry_.child_frame_id = body_id_;
    robot_odometry_.pose.pose.orientation = geometry_quat;
    robot_odometry_.pose.pose.position.x = q_now.x;
    robot_odometry_.pose.pose.position.y = q_now.y;
    robot_odometry_.twist.twist.angular.z = Vb.omega;
    robot_odometry_.twist.twist.linear.x = Vb.x;
    robot_odometry_.twist.twist.linear.y = Vb.y;

    odometry_publisher_->publish(robot_odometry_);

    geometry_msgs::msg::TransformStamped transform_body;

    transform_body.header.stamp = get_clock()->now();
    transform_body.header.frame_id = odom_id_;
    transform_body.child_frame_id = body_id_;
    transform_body.transform.translation.x = x_;
    transform_body.transform.translation.y = y_;
    transform_body.transform.rotation = geometry_quat;

    tf_broadcaster_->sendTransform(transform_body);

    path_.poses.push_back(construct_path_msg());
    path_.header.stamp = get_clock()->now();
    path_publisher_->publish(path_);
  }

  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odometry_publisher_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_publisher_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_subscriber_;
  rclcpp::Service<nuturtle_control::srv::InitialPose>::SharedPtr initial_pose_service_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  turtlelib::DiffDrive turtlebot_;
  nav_msgs::msg::Odometry robot_odometry_;
  nav_msgs::msg::Path path_;
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
  double x_;
  double y_;
  double theta_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Odometry>());
  rclcpp::shutdown();
  return 0;
}
