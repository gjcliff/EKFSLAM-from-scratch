#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_ros/transform_broadcaster.h"

#include "nuturtle_control/srv/initial_pose.hpp"
#include "nuturtle_control/msg/robot_configuration.hpp"

#include "turtlelib/geometry2d.hpp"
#include "turtlelib/diff_drive.hpp"

#include <string>
#include <vector>

using namespace std::chrono_literals;
using std::placeholders::_1, std::placeholders::_2;

/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */

class Odometry : public rclcpp::Node
{
public:
  Odometry()
  : Node("odometry"), count_(0)
  {
    declare_parameter("body_id", "thing");
    declare_parameter("odom_id", "odom");
    declare_parameter("wheel_left", "thing");
    declare_parameter("wheel_right", "thing");

    try {
      body_id_ = get_parameter("body_id").as_string();
    } catch (rclcpp::exceptions::InvalidParameterTypeException()) {
      RCLCPP_ERROR_STREAM(get_logger(), "wtf");
    }
    
    odom_id_ = get_parameter("odom_id").as_string();
    wheel_left_ = get_parameter("wheel_left").as_string();
    wheel_right_ = get_parameter("wheel_right").as_string();

    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    // create publishers
    odometry_publisher_ = create_publisher<nav_msgs::msg::Odometry>(
      "odom", 10);

    // create subscribers
    joint_state_subscriber_ = create_subscription<sensor_msgs::msg::JointState>(
      "joint_states", 10, std::bind(&Odometry::joint_state_callback, this, _1));

    // create services
    initial_pose_service_ = create_service<nuturtle_control::srv::InitialPose>(
      "initial_pose", std::bind(&Odometry::initial_pose_callback, this, _1, _2));

    timer_ = this->create_wall_timer(
      10ms, std::bind(&Odometry::timer_callback, this));

    robot_odometry_.header.frame_id = odom_id_;
    robot_odometry_.child_frame_id = body_id_;
  }

private:
  void initial_pose_callback(
    const std::shared_ptr<nuturtle_control::srv::InitialPose::Request> request,
    std::shared_ptr<nuturtle_control::srv::InitialPose::Response>)
  {
    tf2::Quaternion tf2_quat;
    tf2_quat.setRPY(0.0, 0.0, request->q.theta);

    robot_odometry_.pose.pose.orientation = tf2::toMsg(tf2_quat);
    robot_odometry_.pose.pose.position.x = request->q.x;
    robot_odometry_.pose.pose.position.y = request->q.y;
  }
  void joint_state_callback(const sensor_msgs::msg::JointState msg)
  {

    RCLCPP_INFO_STREAM_ONCE(get_logger(), "joint state callback");
    double phi_l = msg.position.at(0);
    double phi_r = msg.position.at(1);

    turtlelib::Twist2D Vb = turtlebot_.FK(phi_l, phi_r);
    turtlelib::Configuration q_now = turtlebot_.update_configuration(Vb).at(0);

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


  }
  void timer_callback()
  {
    RCLCPP_INFO_STREAM_ONCE(get_logger(), "here");
    turtlelib::Configuration q_now = turtlebot_.get_current_configuration();
    RCLCPP_INFO_STREAM_ONCE(get_logger(), "here");
    geometry_msgs::msg::TransformStamped transform;

    transform.header.stamp = get_clock()->now();
    transform.header.frame_id = odom_id_;
    transform.child_frame_id = body_id_;
    transform.transform.translation.x = q_now.x;
    transform.transform.translation.y = q_now.y;
    transform.transform.translation.z = 0.0;
    transform.transform.rotation.z = q_now.theta;

    tf_broadcaster_->sendTransform(transform);
  }
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odometry_publisher_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_subscriber_;
  rclcpp::Service<nuturtle_control::srv::InitialPose>::SharedPtr initial_pose_service_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  turtlelib::DiffDrive turtlebot_;
  nav_msgs::msg::Odometry robot_odometry_;
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
