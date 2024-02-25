#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <vector>
#include <cmath>
#include <random>
#include <armadillo>

#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/u_int64.hpp"
#include "std_srvs/srv/empty.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "nav_msgs/msg/path.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "nav_msgs/msg/path.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

#include "nuturtlebot_msgs/msg/wheel_commands.hpp"
#include "nuturtlebot_msgs/msg/sensor_data.hpp"
#include "turtlelib/diff_drive.hpp"
#include "nusim/srv/teleport.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1, std::placeholders::_2;

class TurtleSimulation : public rclcpp::Node
{
public:
  TurtleSimulation()
  : Node("nusim")
  {
    // declare parameters
    declare_parameter("rate", 5);
    declare_parameter("x0", 0.0);
    declare_parameter("y0", 0.0);
    declare_parameter("theta0", 0.0);
    declare_parameter("arena_length_x", 1.0);
    declare_parameter("arena_length_y", 1.0);
    declare_parameter("obstacles/x", std::vector<double>({0.0, 0.0}));
    declare_parameter("obstacles/y", std::vector<double>({0.0, 0.0}));
    declare_parameter("obstacles/height", 0.25);
    declare_parameter("obstacles/r", 0.25);
    declare_parameter("draw_only", false);

    declare_parameter("wheel_radius", 0.1);
    declare_parameter("track_width", 1.0);
    declare_parameter("motor_cmd_max", 100);
    declare_parameter("motor_cmd_per_rad_sec", 0.01);
    declare_parameter("encoder_ticks_per_rad", 500.0);
    declare_parameter("collision_radius", 0.2);

    declare_parameter("input_noise", 0.0);
    declare_parameter("slip_fraction", 0.0);
    declare_parameter("basic_sensor_variance", 0.0);
    declare_parameter("max_range", 2.0);

    // set parameters
    rate_ = get_parameter("rate").as_int();
    x_ = get_parameter("x0").as_double();
    y_ = get_parameter("y0").as_double();
    theta_ = get_parameter("theta0").as_double();
    arena_x_length_ = get_parameter("arena_length_x").as_double();
    arena_y_length_ = get_parameter("arena_length_y").as_double();
    obstacles_x_ = get_parameter("obstacles/x").as_double_array();
    obstacles_y_ = get_parameter("obstacles/y").as_double_array();
    obstacle_height_ = get_parameter("obstacles/height").as_double();
    obstacle_radius_ = get_parameter("obstacles/r").as_double();
    draw_only_ = get_parameter("draw_only").as_bool();

    wheel_radius_ = get_parameter("wheel_radius").as_double();
    track_width_ = get_parameter("track_width").as_double();
    motor_cmd_max_ = get_parameter("motor_cmd_max").as_int();
    motor_cmd_per_rad_sec_ = get_parameter("motor_cmd_per_rad_sec").as_double();
    encoder_ticks_per_rad_ = get_parameter("encoder_ticks_per_rad").as_double();
    collision_radius_ = get_parameter("collision_radius").as_double();

    input_noise_ = get_parameter("input_noise").as_double();
    slip_fraction_ = get_parameter("slip_fraction").as_double();
    basic_sensor_variance_ = get_parameter("basic_sensor_variance").as_double();
    max_range_ = get_parameter("max_range").as_double();

    input_noise_generator_ = std::normal_distribution<>(0.0, input_noise_);
    slip_fraction_generator_ = std::uniform_real_distribution(-slip_fraction_, slip_fraction_);
    sensor_variance_generator_ = std::normal_distribution<>(0.0, basic_sensor_variance_);

    turtlelib::RobotDimensions rd{0.0, track_width_ / 2, wheel_radius_};
    turtlebot_.set_robot_dimensions(rd);

    // set miscellaneous variables
    og_x_ = x_;
    og_y_ = y_;
    og_theta_ = theta_;

    // check whether or not the obstacle arrays are the same size
    if (obstacles_x_.size() != obstacles_y_.size()) {
      RCLCPP_ERROR_STREAM(get_logger(), "obstacles_x_ and obstacles_y_ are not the same size");
      rclcpp::shutdown();
    }

    auto qos = rclcpp::QoS(rclcpp::KeepLast(10)).transient_local();

    // declare publishers
    timestep_publisher_ = create_publisher<std_msgs::msg::UInt64>("~/timestep", qos);
    walls_publisher_ = create_publisher<visualization_msgs::msg::MarkerArray>("~/walls", qos);
    obstacles_publisher_ = create_publisher<visualization_msgs::msg::MarkerArray>(
      "~/obstacles", qos);
    fake_obstacles_publisher_ = create_publisher<visualization_msgs::msg::MarkerArray>(
        "/fake_sensor", qos);
    sensor_data_publisher_ = create_publisher<nuturtlebot_msgs::msg::SensorData>("sensor_data", 10);
    path_publisher_ = create_publisher<nav_msgs::msg::Path>("~/path", 10);
    laser_scan_publisher_ = create_publisher<sensor_msgs::msg::LaserScan>("~/scan", 10);

    // declare subscribers
    wheel_commands_subscriber_ = create_subscription<nuturtlebot_msgs::msg::WheelCommands>(
      "wheel_cmd", 10, std::bind(&TurtleSimulation::wheel_commands_callback, this, _1));

    // declare static transform broadcaster
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    // declare services
    reset_service_ =
      create_service<std_srvs::srv::Empty>(
      "reset",
      std::bind(&TurtleSimulation::reset_callback, this, _1, _2));
    teleport_service_ =
      create_service<nusim::srv::Teleport>(
      "teleport",
      std::bind(&TurtleSimulation::teleport_callback, this, _1, _2));

    // initialize the transform broadcaster
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    tf_broadcaster_->sendTransform(construct_transform_msg(x_, y_, theta_));
    timer_ = create_wall_timer(
      static_cast<std::chrono::milliseconds>(rate_),
      std::bind(&TurtleSimulation::timer_callback, this));
    slow_timer_ = create_wall_timer(
      200ms,
      std::bind(&TurtleSimulation::slow_timer_callback, this));

    visualization_msgs::msg::MarkerArray wall_array = construct_wall_array(red);
    walls_publisher_->publish(wall_array);

  }

private:
  struct Color
  {
    double r;
    double g;
    double b;
  };

  std::mt19937 & get_random()
  {
    // static variables inside a function are created once and persist for the remainder of the program
    static std::random_device rd{}; 
    static std::mt19937 mt{rd()};
    // we return a reference to the pseudo-random number genrator object. This is always the
    // same object every time get_random is called
    return mt;
  }

  visualization_msgs::msg::MarkerArray construct_obstacle_array(vector<double> marker_array_x, vector<double> marker_array_y, Color c)
  {
    visualization_msgs::msg::MarkerArray arr;
    for (int i = 0; i < (int)obstacles_x_.size(); i++) {
      visualization_msgs::msg::Marker marker;
      marker.header.frame_id = "nusim/world";
      marker.header.stamp = get_clock()->now();
      marker.id = i;
      marker.type = visualization_msgs::msg::Marker::CYLINDER;
      marker.action = visualization_msgs::msg::Marker::ADD;
      marker.pose.position.x = marker_array_x[i];
      marker.pose.position.y = marker_array_y[i];
      marker.pose.position.z = wall_height_ / 2;
      marker.pose.orientation.x = 0;
      marker.pose.orientation.y = 0;
      marker.pose.orientation.z = 0;
      marker.pose.orientation.w = 1.0;
      marker.scale.x = obstacle_radius_;
      marker.scale.y = obstacle_radius_;
      marker.scale.z = obstacle_height_;
      marker.color.a = 1.0;
      marker.color.r = c.r;
      marker.color.g = c.g;
      marker.color.b = c.b;

      arma::vec2 tmp = {marker_array_x.at(i) - x_, marker_array_y.at(i) - y_};
      double dist = arma::norm(tmp);
      if (dist > max_range_) {
        marker.action = visualization_msgs::msg::Marker::DELETE;
      }

      arr.markers.push_back(marker);
    }

    return arr;
  }

  visualization_msgs::msg::Marker construct_marker(
    double pos_x, double pos_y, double scale_x,
    double scale_y, int id,
    Color c)
  {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "nusim/world";
    marker.header.stamp = get_clock()->now();
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
    marker.color.r = c.r;
    marker.color.g = c.g;
    marker.color.b = c.b;
    return marker;
  }

  visualization_msgs::msg::MarkerArray construct_wall_array(Color c)
  {
    visualization_msgs::msg::MarkerArray wall_array;

    double x_length = (arena_x_length_ + wall_thickness_);
    double y_length = (arena_y_length_ + wall_thickness_);

    wall_array.markers.push_back(
      construct_marker(
        arena_x_length_ / 2, 0.0, wall_thickness_,
        x_length, 0, c));
    wall_array.markers.push_back(
      construct_marker(
        0.0, arena_y_length_ / 2, y_length,
        wall_thickness_, 1, c));
    wall_array.markers.push_back(
      construct_marker(
        -arena_x_length_ / 2, 0.0, wall_thickness_,
        x_length, 2, c));
    wall_array.markers.push_back(
      construct_marker(
        0.0, -arena_y_length_ / 2, y_length,
        wall_thickness_, 1, c));

    return wall_array;
  }

  geometry_msgs::msg::TransformStamped construct_transform_msg(double x, double y, double theta)
  {
    geometry_msgs::msg::TransformStamped t;

    t.header.stamp = get_clock()->now();
    t.header.frame_id = "nusim/world";
    t.child_frame_id = "red/base_footprint";

    t.transform.translation.x = x;
    t.transform.translation.y = y;
    t.transform.translation.z = 0.0;

    tf2::Quaternion q_tf2;
    q_tf2.setRPY(0, 0, theta);

    geometry_msgs::msg::Quaternion q = tf2::toMsg(q_tf2);

    t.transform.rotation.x = q.x;
    t.transform.rotation.y = q.y;
    t.transform.rotation.z = q.z;
    t.transform.rotation.w = q.w;

    return t;
  }
  
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

  void teleport_callback(
    const std::shared_ptr<nusim::srv::Teleport::Request> request,
    std::shared_ptr<nusim::srv::Teleport::Response> response)
  {
    geometry_msgs::msg::TransformStamped t = construct_transform_msg(
      request->x, request->y,
      request->theta);

    tf_broadcaster_->sendTransform(t);
    (void) response;

    x_ = request->x;
    y_ = request->y;
    theta_ = request->theta;
  }

  void reset_callback(
    const std::shared_ptr<std_srvs::srv::Empty::Request>,
    std::shared_ptr<std_srvs::srv::Empty::Response>)
  {
    current_timestep_ = 0;
    x_ = og_x_;
    y_ = og_y_;
    theta_ = og_theta_;
  }

  void wheel_commands_callback(const nuturtlebot_msgs::msg::WheelCommands msg)
  {
    if (msg.left_velocity == 0 && msg.right_velocity == 0){
      left_wheel_velocity_ = msg.left_velocity * motor_cmd_per_rad_sec_ / (1000.0 / rate_);
      right_wheel_velocity_ = msg.right_velocity * motor_cmd_per_rad_sec_ / (1000.0 / rate_);
    } else {
      double noise = input_noise_generator_(get_random());
      left_wheel_velocity_ = msg.left_velocity * motor_cmd_per_rad_sec_ / (1000.0 / rate_);
      right_wheel_velocity_ = msg.right_velocity * motor_cmd_per_rad_sec_ / (1000.0 / rate_) + noise;

      left_wheel_velocity_ *= (1 + slip_fraction_generator_(get_random()));
      right_wheel_velocity_ *= (1 + slip_fraction_generator_(get_random()));
    }
  }

  void timer_callback()
  {
    if (draw_only_) {
      // keep track of the current timestep
      current_timestep_ += 1;
      auto timestep_message = std_msgs::msg::UInt64();
      timestep_message.data = current_timestep_;
      timestep_publisher_->publish(timestep_message);

      visualization_msgs::msg::MarkerArray wall_array = construct_wall_array(red);
      walls_publisher_->publish(wall_array);

      visualization_msgs::msg::MarkerArray obstacle_array = construct_obstacle_array(obstacles_x_, obstacles_y_, red);
      obstacles_publisher_->publish(obstacle_array);
    } else {
      // keep track of the current timestep
      current_timestep_ += 1;
      auto timestep_message = std_msgs::msg::UInt64();
      timestep_message.data = current_timestep_;
      timestep_publisher_->publish(timestep_message);
      
      // add the walls and obstacles to the world
      visualization_msgs::msg::MarkerArray wall_array = construct_wall_array(red);
      walls_publisher_->publish(wall_array);

      visualization_msgs::msg::MarkerArray obstacle_array = construct_obstacle_array(obstacles_x_, obstacles_y_, red);
      obstacles_publisher_->publish(obstacle_array);

      // get the body twist
      turtlelib::Twist2D Vb = turtlebot_.FK(left_wheel_velocity_, right_wheel_velocity_);

      // calculate the current encoder ticks
      left_encoder_ticks_ += left_wheel_velocity_ * encoder_ticks_per_rad_;
      right_encoder_ticks_ += right_wheel_velocity_ * encoder_ticks_per_rad_;

      int rounded_left_encoder_ticks_ = static_cast<int>(std::round(left_encoder_ticks_));
      int rounded_right_encoder_ticks_ = static_cast<int>(std::round(right_encoder_ticks_));

      // update the configuration of the red robot in the world frame
      turtlelib::Configuration qv = turtlebot_.update_configuration(Vb);
      x_ = qv.x;
      y_ = qv.y;
      theta_ = qv.theta;

      // broadcast the position of the red robot in the world frame
      geometry_msgs::msg::TransformStamped t = construct_transform_msg(x_, y_, theta_);
      tf_broadcaster_->sendTransform(t);

      // publish the path of the red robot
      path_.poses.push_back(construct_path_msg());
      path_.header.stamp = get_clock()->now();
      path_publisher_->publish(path_);

      // publish the encoder data
      nuturtlebot_msgs::msg::SensorData sensor_data_msg;
      sensor_data_msg.left_encoder = rounded_left_encoder_ticks_;
      sensor_data_msg.right_encoder = rounded_right_encoder_ticks_;
      sensor_data_msg.stamp = get_clock()->now();

      sensor_data_publisher_->publish(sensor_data_msg);
    }
  }

  void slow_timer_callback()
  {
    double noise;
    for (int i = 0; i < (int)obstacles_x_.size(); i++) {
      noise = sensor_variance_generator_(get_random());
      fake_obstacles_x_.at(i) = obstacles_x_.at(i) + noise;
      noise = sensor_variance_generator_(get_random());
      fake_obstacles_y_.at(i) = obstacles_y_.at(i) + noise;
    }
    visualization_msgs::msg::MarkerArray relative_obstacle_array = construct_obstacle_array(fake_obstacles_x_, fake_obstacles_y_, yellow);
    fake_obstacles_publisher_->publish(relative_obstacle_array);
  }
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::TimerBase::SharedPtr slow_timer_;
  // publishers
  rclcpp::Publisher<std_msgs::msg::UInt64>::SharedPtr timestep_publisher_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr walls_publisher_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr obstacles_publisher_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr fake_obstacles_publisher_;
  rclcpp::Publisher<nuturtlebot_msgs::msg::SensorData>::SharedPtr sensor_data_publisher_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr laser_scan_publisher_;
  // subscribers
  rclcpp::Subscription<nuturtlebot_msgs::msg::WheelCommands>::SharedPtr wheel_commands_subscriber_;
  // transform broadcaster
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  // services
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr reset_service_;
  rclcpp::Service<nusim::srv::Teleport>::SharedPtr teleport_service_;

  turtlelib::DiffDrive turtlebot_;
  nav_msgs::msg::Path path_;

  Color red = {1.0, 0.0, 0.0};
  Color green = {0.0, 1.0, 0.0};
  Color blue = {0.0, 0.0, 1.0};
  Color yellow = {1.0, 1.0, 0.0};
  Color purple = {1.0, 0.0, 1.0};

  std::normal_distribution<> input_noise_generator_;
  std::uniform_real_distribution<> slip_fraction_generator_;
  std::normal_distribution<> sensor_variance_generator_;

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
  double wheel_radius_;
  double track_width_;
  double input_noise_;
  double slip_fraction_;
  double basic_sensor_variance_;
  double max_range_;
  bool draw_only_;

  int motor_cmd_max_;
  double motor_cmd_per_rad_sec_;
  double encoder_ticks_per_rad_;
  double collision_radius_;
  double left_encoder_ticks_;
  double right_encoder_ticks_;
  double left_wheel_velocity_;
  double right_wheel_velocity_;
  std::vector<double> obstacles_x_;
  std::vector<double> obstacles_y_;
  std::vector<double> fake_obstacles_x_;
  std::vector<double> fake_obstacles_y_;
  double obstacle_radius_;
  double obstacle_height_ = 0.25;
  unsigned int current_timestep_ = 0;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TurtleSimulation>());
  rclcpp::shutdown();
  return 0;
}
