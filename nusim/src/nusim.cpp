#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2/LinearMath/Quaternion.h"

#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/u_int64.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "std_srvs/srv/empty.hpp"
#include "nusim/srv/teleport.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1, std::placeholders::_2;

class TurtleSimulation : public rclcpp::Node
{
public:
  TurtleSimulation()
  : Node("minimal_publisher"), count_(0)
  {
    // declare parameters
    this->declare_parameter("rate", 5);
    this->declare_parameter("x0", 0.0);
    this->declare_parameter("y0", 0.0);
    this->declare_parameter("theta0", 0.0);

    // set parameters
    x_ = this->get_parameter("x0").as_double();
    y_ = this->get_parameter("y0").as_double();
    theta_ = this->get_parameter("theta0").as_double();

    // declare publisher
    timestep_publisher_ = this->create_publisher<std_msgs::msg::UInt64>("~/timestep", 10);

    // declare static transform broadcaster
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    // declare services
    reset_service_ =
      this->create_service<std_srvs::srv::Empty>(
      "reset",
      std::bind(&TurtleSimulation::reset_callback, this, _1, _2));
    teleport_service_ =
      this->create_service<nusim::srv::Teleport>(
      "reset",
      std::bind(&TurtleSimulation::teleport_callback, this, _1, _2));
    // teleport_service_ = this->create_service<geometry_msgs::srv::

    // initialize the transform broadcaster
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    tf_broadcaster_->sendTransform(construct_transform_msg(x_, y_, theta_));
    timer_ = this->create_wall_timer(
      (std::chrono::milliseconds)this->get_parameter("rate").as_int(),
      std::bind(&TurtleSimulation::timer_callback, this));
  }

private:
  geometry_msgs::msg::TransformStamped construct_transform_msg(double x, double y, double theta)
  {
    geometry_msgs::msg::TransformStamped t;

    t.header.stamp = this->get_clock()->now();
    t.header.frame_id = "nusim/world";
    t.child_frame_id = "red/base_footprint";

    t.transform.translation.x = x;
    t.transform.translation.y = y;
    t.transform.translation.z = 0.0;

    tf2::Quaternion q;
    q.setRPY(0, 0, theta);
    t.transform.rotation.x = q.x();
    t.transform.rotation.y = q.y();
    t.transform.rotation.z = q.z();
    t.transform.rotation.w = q.w();

    return t;
  }

  void teleport_callback(
    const std::shared_ptr<nusim::srv::Teleport::Request> request,
    std::shared_ptr<nusim::srv::Teleport::Response> response)
  {
    geometry_msgs::msg::TransformStamped t = construct_transform_msg(
      request->x, request->y,
      request->theta);

    tf_broadcaster_->sendTransform(t);
    (void) response;
  }

  void reset_callback(
    const std::shared_ptr<std_srvs::srv::Empty::Request> request,
    std::shared_ptr<std_srvs::srv::Empty::Response> response)
  {
    (void) request; // is this good practice????
    (void) response; // found here: https://docs.ros.org/en/humble/p/rclcpp/generated/program_listing_file_include_rclcpp_any_subscription_callback.hpp.html
    current_timestep_ = 0;
    x_ = 0.0;
    y_ = 0.0;
    theta_ = 0.0;
  }

  void timer_callback()
  {
    // RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", timestep_message.data.c_str());
    geometry_msgs::msg::TransformStamped t = construct_transform_msg(x_, y_, theta_);
    tf_broadcaster_->sendTransform(t);

    // keep track of the current timestep
    current_timestep_ += 1;
    auto timestep_message = std_msgs::msg::UInt64();
    timestep_message.data = current_timestep_;
    timestep_publisher_->publish(timestep_message);
  }
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::UInt64>::SharedPtr timestep_publisher_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr reset_service_;
  rclcpp::Service<nusim::srv::Teleport>::SharedPtr teleport_service_;

  double x_ = 0.0;
  double y_ = 0.0;
  double theta_ = 0.0;
  size_t count_;
  unsigned int current_timestep_ = 0;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TurtleSimulation>());
  rclcpp::shutdown();
  return 0;
}
