#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2/LinearMath/Quaternion.h"

#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/u_int64.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "std_srvs/srv/empty.hpp"
#include "nusim/srv/teleport.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "visualization_msgs/msg/marker.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1, std::placeholders::_2;

class TurtleSimulation : public rclcpp::Node
{
public:
  TurtleSimulation()
  : Node("nusim"), count_(0)
  {
    // declare parameters
      // unnecessary use of this-> (all uses here are unnecessary
    this->declare_parameter("rate", 5);
    this->declare_parameter("x0", 0.0);
    this->declare_parameter("y0", 0.0);
    this->declare_parameter("theta0", 0.0);
    this->declare_parameter("arena_length_x", 1.0);
    this->declare_parameter("arena_length_y", 1.0);
    this->declare_parameter("obstacles/x", std::vector<double>({0.0, 0.0}));
    this->declare_parameter("obstacles/y", std::vector<double>({0.0, 0.0}));
    this->declare_parameter("obstacles/height", 0.25);
    this->declare_parameter("obstacles/r", 0.25);

    // set parameters
    rate_ = this->get_parameter("rate").as_int();
    x_ = this->get_parameter("x0").as_double();
    y_ = this->get_parameter("y0").as_double();
    theta_ = this->get_parameter("theta0").as_double();
    arena_x_length_ = this->get_parameter("arena_length_x").as_double();
    arena_y_length_ = this->get_parameter("arena_length_y").as_double();
    obstacles_x = this->get_parameter("obstacles/x").as_double_array();
    obstacles_y = this->get_parameter("obstacles/y").as_double_array();
    obstacle_height_ = this->get_parameter("obstacles/height").as_double();
    obstacle_radius_ = this->get_parameter("obstacles/r").as_double();

    // set miscellaneous variables
    og_x_ = x_;
    og_y_ = y_;
    og_theta_ = theta_;

    // check whether or not the obstacle arrays are the same size
    if (obstacles_x.size() != obstacles_y.size()) {
      RCLCPP_ERROR(this->get_logger(), "obstacles_x and obstacles_y are not the same size");
      rclcpp::shutdown(); // throw an exception not shutdown
    }

    auto qos = rclcpp::QoS(rclcpp::KeepLast(10)).transient_local();

    // declare publisher
    timestep_publisher_ = this->create_publisher<std_msgs::msg::UInt64>("~/timestep", qos);
    walls_publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("~/walls", qos);
    obstacles_publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
      "~/obstacles", qos);

    // declare static transform broadcaster
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    // declare services
    reset_service_ =
      this->create_service<std_srvs::srv::Empty>(
      "reset",
      std::bind(&TurtleSimulation::reset_callback, this, _1, _2));
    teleport_service_ =
      this->create_service<nusim::srv::Teleport>(
      "teleport",
      std::bind(&TurtleSimulation::teleport_callback, this, _1, _2));
    // teleport_service_ = this->create_service<geometry_msgs::srv::

    // initialize the transform broadcaster
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    tf_broadcaster_->sendTransform(construct_transform_msg(x_, y_, theta_));
    timer_ = this->create_wall_timer(
      (std::chrono::milliseconds)this->get_parameter("rate").as_int(),
      std::bind(&TurtleSimulation::timer_callback, this));

    visualization_msgs::msg::MarkerArray wall_array = construct_wall_array();
    walls_publisher_->publish(wall_array);

  }

private:
  visualization_msgs::msg::MarkerArray construct_obstacle_array()
  {
    visualization_msgs::msg::MarkerArray arr;
    for (int i = 0; i < (int)obstacles_x.size(); i++) { // i is size_t, don't use C style cast (int)
      visualization_msgs::msg::Marker marker;
      marker.header.frame_id = "nusim/world";
      marker.header.stamp = this->get_clock()->now();
      marker.id = i;
      marker.type = visualization_msgs::msg::Marker::CYLINDER;
      marker.action = visualization_msgs::msg::Marker::ADD;
      marker.pose.position.x = obstacles_x[i]; // .at()
      marker.pose.position.y = obstacles_y[i];
      marker.pose.position.z = wall_height_ / 2;
      marker.pose.orientation.x = 0;
      marker.pose.orientation.y = 0;
      marker.pose.orientation.z = 0;
      marker.pose.orientation.w = 1.0;
      marker.scale.x = obstacle_radius_;
      marker.scale.y = obstacle_radius_;
      marker.scale.z = obstacle_height_;
      marker.color.a = 1.0;
      marker.color.r = 1.0;

      arr.markers.push_back(marker);
    }

    return arr;
  }

  visualization_msgs::msg::Marker construct_marker(
    double pos_x, double pos_y, double scale_x,
    double scale_y, int id)
  {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "nusim/world";
    marker.header.stamp = this->get_clock()->now();
    marker.id = id;
    marker.type = visualization_msgs::msg::Marker::CUBE;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.position.x = pos_x;
    marker.pose.position.y = pos_y;
    marker.pose.position.z = wall_height_ / 2;
    marker.pose.orientation.x = 0;
    marker.pose.orientation.y = 0;
    marker.pose.orientation.z = 0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = scale_x;
    marker.scale.y = scale_y;
    marker.scale.z = wall_height_;
    marker.color.a = 1.0;
    marker.color.r = 1.0;
    return marker;
  }

  visualization_msgs::msg::MarkerArray construct_wall_array()
  {
    visualization_msgs::msg::MarkerArray wall_array;

    double x_length = (arena_x_length_ + wall_thickness_);
    double y_length = (arena_y_length_ + wall_thickness_);

    wall_array.markers.push_back(
      construct_marker(
        arena_x_length_ / 2, 0.0, wall_thickness_,
        x_length, 0));
    wall_array.markers.push_back(
      construct_marker(
        0.0, arena_y_length_ / 2, y_length,
        wall_thickness_, 1));
    wall_array.markers.push_back(
      construct_marker(
        -arena_x_length_ / 2, 0.0, wall_thickness_,
        x_length, 2));
    wall_array.markers.push_back(
      construct_marker(
        0.0, -arena_y_length_ / 2, y_length,
        wall_thickness_, 3));

    return wall_array;
  }

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
    (void) response; // this is C, in C++ you would omit the name of the parameter

    x_ = request->x;
    y_ = request->y;
    theta_ = request->theta;
  }

  void reset_callback(
    const std::shared_ptr<std_srvs::srv::Empty::Request> request,
    std::shared_ptr<std_srvs::srv::Empty::Response> response)
  {
    (void) request; // is this good practice????
    (void) response; // found here: https://docs.ros.org/en/humble/p/rclcpp/generated/program_listing_file_include_rclcpp_any_subscription_callback.hpp.html
    // No the above is C, in C++ you would omit the parameter
    current_timestep_ = 0;
    x_ = og_x_;
    y_ = og_y_;
    theta_ = og_theta_;
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

    visualization_msgs::msg::MarkerArray wall_array = construct_wall_array();
    walls_publisher_->publish(wall_array);

    visualization_msgs::msg::MarkerArray obstacle_array = construct_obstacle_array();
    obstacles_publisher_->publish(obstacle_array);
  }
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::UInt64>::SharedPtr timestep_publisher_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr walls_publisher_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr obstacles_publisher_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr reset_service_;
  rclcpp::Service<nusim::srv::Teleport>::SharedPtr teleport_service_;

  int rate_ = 0;
  double x_ = 0.0;
  double y_ = 0.0;
  double theta_ = 0.0;
  double og_x_;
  double og_y_;
  double og_theta_;
  double arena_x_length_;
  double arena_y_length_;
  double wall_height_ = 0.25;
  double wall_thickness_ = 0.05;
  std::vector<double> obstacles_x;
  std::vector<double> obstacles_y;
  double obstacle_radius_;
  double obstacle_height_ = 0.25;
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
