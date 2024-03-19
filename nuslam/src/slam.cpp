#include <chrono>
#include <rclcpp/logging.hpp>
#include <string>
#include <vector>
#include <cmath>
#include <random>
#include <armadillo>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/header.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "visualization_msgs/msg/marker_array.hpp"
#include "nuslam/msg/circle.hpp"
#include "nuslam/msg/landmarks.hpp"

#include "turtlelib/se2d.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

class Slam : public rclcpp::Node
{
public:
  Slam()
  : Node("slam")
  {
    declare_parameter("use_fake_sensors", true);
    use_fake_sensors = get_parameter("use_fake_sensors").as_bool();
    odometry_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "/green/odom", 10, std::bind(&Slam::odometry_callback, this, _1));

    path_publisher_ = create_publisher<nav_msgs::msg::Path>(
      "slam_path", 10);

    landmark_publisher_ = create_publisher<visualization_msgs::msg::MarkerArray>(
      "landmark_markers", 10);

    fake_sensor_sub_ = create_subscription<visualization_msgs::msg::MarkerArray>(
      "/fake_sensor", 10, std::bind(&Slam::fake_sensor_callback, this, _1));

    landmark_sub_ = create_subscription<nuslam::msg::Landmarks>(
      "landmarks", 10, std::bind(&Slam::landmark_callback, this, _1));

    // initialize the transform broadcaster
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    n_ = 0;
    Qfactor_ = 2.0;
    Rfactor_ = 100.0;
    sigma_diag_ = 10000000.0;
    R_ = arma::eye(2, 2) * Rfactor_;
    
    initialize_xi();
    initialize_Q();
    initialize_sigma();

    path_.header.frame_id = "nusim/world";
  }

private:
  void publish_landmark(const nuslam::msg::Circle & landmark, int id)
  {
    visualization_msgs::msg::MarkerArray landmark_markers;
    visualization_msgs::msg::Marker landmark_marker;
    landmark_marker.header.stamp = time;
    landmark_marker.header.frame_id = "red/base_footprint";
    landmark_marker.id = id;
    landmark_marker.type = visualization_msgs::msg::Marker::CYLINDER;
    landmark_marker.action = visualization_msgs::msg::Marker::ADD;
    landmark_marker.pose.position.x = landmark.x;
    landmark_marker.pose.position.y = landmark.y;
    landmark_marker.pose.position.z = 0.125;
    landmark_marker.scale.x = landmark.radius * 2;
    landmark_marker.scale.y = landmark.radius * 2;
    landmark_marker.scale.z = 0.25;
    landmark_marker.color.a = 1.0;
    landmark_marker.color.r = 0.0;
    landmark_marker.color.g = 1.0;
    landmark_marker.color.b = 0.0;
    landmark_marker.lifetime = rclcpp::Duration::from_seconds(1.0);
    landmark_markers.markers.push_back(landmark_marker);

    landmark_publisher_->publish(landmark_markers);
  }

  int calculate_mahalanobis_distance(const nuslam::msg::Circle & landmark)
    // returns the map index of the landmark we are closest to
  {
    // I'M LEAVING THESE IN because I want to come back in the spring quarter
    // and fix a bug that occurs like 10% of the time.
    // RCLCPP_INFO_STREAM(get_logger(), "map_: " << map_);
    // if the map's empty, just add the landmark to the map. We have nothing
    // to compare it to anyways
    // RCLCPP_INFO_STREAM(get_logger(), "map_.size(): " << map_.size());
    // RCLCPP_INFO_STREAM(get_logger(), "landmark.x: " << landmark.x);
    // RCLCPP_INFO_STREAM(get_logger(), "landmark.y: " << landmark.y);
    // put the landmark's postiion into the map frame
    turtlelib::Point2D m = map_to_robot_(turtlelib::Point2D{landmark.x,
        landmark.y});
    // RCLCPP_INFO_STREAM(get_logger(), "m: " << m.x << ", " << m.y << "\n");
    arma::colvec tmp = {m.x, m.y};

    // for this landmark, calculate the mahalanobis distance between it and
    // the landmarks in the map
    std::vector<double> mahalanobis_distances;
    for (unsigned int j = 0; j < map_.size()/2; j++) {
      // int n_tmp = n_ + 1;
      arma::mat H_k = calc_H(m.x, m.y, j, n_);
      arma::mat sigma_tmp = sigma_;
      arma::mat sigma_k = H_k * sigma_ * H_k.t() + R_;
      arma::mat z_j = get_range_bearing_measurement(m.x, m.y);
      arma::mat z_hat_k = get_range_bearing_measurement(map_(j*2), map_(j*2+1));
      arma::mat d_k = (z_j - z_hat_k).t() * arma::inv(sigma_k) * (z_j - z_hat_k);
      // RCLCPP_INFO_STREAM(get_logger(), "d_k: " << d_k(0,0));
      mahalanobis_distances.push_back(d_k(0,0));
    }

    // if the mahalanobis distance is greater than the threshold for all
    // obstacles in the map, then we can assume this is a new landmark
    bool is_new_landmark = true;
    int map_index = -1;
    double min_distance = 10000000;
    for (unsigned int j = 0; j < mahalanobis_distances.size(); j++) {
      if (mahalanobis_distances.at(j) < min_distance) {
        min_distance = mahalanobis_distances.at(j);
        map_index = j*2;
      }

      if (mahalanobis_distances.at(j) < 0.01) {
        is_new_landmark = false;
      }

      // check the euclidean distance
      if (std::sqrt(std::pow(m.x - map_(j*2), 2) + std::pow(m.y - map_(j*2+1), 2)) < 0.05) {
        is_new_landmark = false;
      }
    }

    // if it's a new landmark, add it to the map
    if (is_new_landmark) {
      RCLCPP_INFO_STREAM(get_logger(), "New landmark detected at: " << m.x << ", " << m.y);
      map_ = arma::join_vert(map_, tmp);
      adjust_matrices(map_.size()/2);
      map_index = 2*(map_.size()/2-1);
    }
    RCLCPP_INFO_STREAM(get_logger(), "");
    return map_index;
  }

  arma::mat calc_H(double mx, double my, int j, int n)
  {
    double delta_x = mx - xi_(1);
    double delta_y = my - xi_(2);
    double d = std::pow(delta_x, 2) + std::pow(delta_y, 2);

    arma::mat block1{{0, -delta_x / std::sqrt(d), -delta_y / std::sqrt(d)},
      {-1, delta_y / d, -delta_x / d}};
    arma::mat block2 = arma::join_vert(arma::zeros(1, 2 * (j)), arma::zeros(1, 2 * (j)));
    arma::mat block3{{delta_x / std::sqrt(d), delta_y / std::sqrt(d)},
      {-delta_y / d, delta_x / d}};
    arma::mat block4 =
      arma::join_vert(arma::zeros(1, 2 * n - 2 * (j + 1)), arma::zeros(1, 2 * n - 2 * (j + 1)));

    arma::mat block12 = arma::join_horiz(block1, block2);
    arma::mat block34 = arma::join_horiz(block3, block4);
    arma::mat H = arma::join_horiz(block12, block34);

    return H;
  }

  void adjust_matrices(const int n) {
    n_ = n;
    // RCLCPP_INFO_STREAM(get_logger(), "n_ is: " << n_);

    // adjust xi
    if (xi_.size() > 3) {
      xi_.shed_rows(3,xi_.size()-1);
    }
    xi_ = arma::join_vert(xi_, map_);
    
    // adjust Qbar
    arma::mat Qbar_tmp = Qbar_;
    int n_diff = n_ - (Qbar_.n_cols-3)/2.0;
    Qbar_tmp = arma::join_horiz(Qbar_tmp, arma::zeros(Qbar_.n_rows, 2 * n_diff));
    Qbar_tmp = arma::join_vert(Qbar_tmp, arma::zeros(2 * n_diff, Qbar_.n_cols + 2 * n_diff));
    Qbar_ = Qbar_tmp;

    // adjust sigma
    arma::mat sigma_tmp = sigma_;
    n_diff = n_ - (sigma_.n_cols-3)/2.0;
    sigma_tmp = arma::join_horiz(sigma_tmp, arma::zeros(sigma_.n_rows, 2 * n_diff));
    sigma_tmp = arma::join_vert(sigma_tmp, arma::join_horiz(arma::zeros(2 * n_diff, sigma_.n_cols),arma::eye(2 * n_diff, 2 * n_diff)*sigma_diag_));
    sigma_ = sigma_tmp;
  }

  void adjust_matrices(const visualization_msgs::msg::MarkerArray & msg) {
    n_ = msg.markers.size();

    // adjust map
    map_ = arma::zeros(msg.markers.size()*2, 1);
    map_ids_.clear();
    for (int j = 0; j < static_cast<int>(msg.markers.size()); j++) {
      turtlelib::Point2D m = map_to_robot_(turtlelib::Point2D{msg.markers.at(j).pose.position.x,
        msg.markers.at(j).pose.position.y});
      map_ids_.push_back(msg.markers.at(j).id);
      map_(j*2) = m.x;
      map_(j*2+1) = m.y;
    }
    
    // adjust xi
    if(xi_.size() > 3) {
      xi_.shed_rows(3,xi_.size()-1);
    }
    xi_ = arma::join_vert(xi_, map_);
    
    // adjust Qbar
    arma::mat Qbar_tmp = Qbar_;
    Qbar_tmp = arma::join_horiz(Qbar_tmp, arma::zeros(Qbar_.n_rows, 2 * (3 + n_ - Qbar_.n_cols)));
    Qbar_tmp = arma::join_vert(Qbar_tmp, arma::zeros(2 * (3+n_ - Qbar_.n_rows), Qbar_.n_cols + 2 * (3+n_ - Qbar_.n_cols)));
    Qbar_ = Qbar_tmp;

    // adjust sigma
    arma::mat sigma_tmp = sigma_;
    sigma_tmp = arma::join_horiz(sigma_tmp, arma::zeros(sigma_.n_rows, 2 * (3+n_ - sigma_.n_cols)));
    sigma_tmp = arma::join_vert(sigma_tmp, arma::join_horiz(arma::zeros(2 * (3+n_ - sigma_.n_rows),sigma_.n_cols),arma::eye(2*(3+n_-sigma_.n_rows),2*(3+n_-sigma_.n_cols))*sigma_diag_));
    sigma_ = sigma_tmp;
  }

  void landmark_callback(const nuslam::msg::Landmarks & msg)
  {
    time = get_clock()->now();
    if(!use_fake_sensors) {

      for (int j = 0; j < static_cast<int>(msg.landmarks.size()); j++) {
        // RCLCPP_INFO_STREAM(get_logger(), "msg.landmarks.size(): " << msg.landmarks.size());
        // RCLCPP_INFO_STREAM(get_logger(), "landmark: " << msg.landmarks.at(j).x << ", " << msg.landmarks.at(j).y);
        int map_index = calculate_mahalanobis_distance(msg.landmarks.at(j));
        // RCLCPP_INFO_STREAM(get_logger(), "map_index: " << map_index);
        publish_landmark(msg.landmarks.at(j), map_index/2);
        // RCLCPP_INFO_STREAM(get_logger(), "published landmark");
        // this is the location of the marker in the robot's frame
        turtlelib::Point2D m = map_to_robot_(turtlelib::Point2D{msg.landmarks.at(j).x,
            msg.landmarks.at(j).y});

        // update the measurement model
        // RCLCPP_INFO_STREAM(get_logger(), "calculating Hj");
        arma::mat Hj = calc_H(m.x, m.y, j);
        // RCLCPP_INFO_STREAM(get_logger(), "done calculating Hj");


        // perform the SLAM update

        // calculate the Kalman gain
        arma::mat K = sigma_ * Hj.t() * arma::inv(Hj * sigma_ * Hj.t() + R_);

        // calculate the position of the robot based on current measurements
        arma::colvec z = get_range_bearing_measurement(m.x, m.y);

        // calculate the position of the robot based on previous measurements
        arma::colvec z_hat = get_range_bearing_measurement(map_(map_index), map_(map_index+1));

        arma::mat z_diff = arma::zeros(2, 1);
        z_diff(0) = z(0) - z_hat(0);
        z_diff(1) = turtlelib::normalize_angle(z(1) - z_hat(1));

        // calculate the correction to the robot's position
        correction_ = K * (z_diff);
        turtlelib::Transform2D map_to_odom_new{{correction_(1), correction_(2)}, correction_(0)};
        map_to_odom_ = map_to_odom_new * map_to_odom_;

        xi_ = xi_ + correction_;

        // compute the posterior covariance
        sigma_ = (arma::eye(xi_.size(), xi_.size()) - K * Hj) * sigma_;

        xi_.shed_rows(3,xi_.size()-1);
        xi_ = arma::join_vert(xi_, map_);

        // correct the position of the robot in the map frame by updating
        // the transform from map to odom

        geometry_msgs::msg::TransformStamped t;
        t.header.stamp = time;
        t.header.frame_id = "map";
        t.child_frame_id = "green/odom";

        tf2::Quaternion q_correct;
        q_correct.setRPY(0, 0, map_to_odom_.rotation());

        t.transform.translation.x = map_to_odom_.translation().x;
        t.transform.translation.y = map_to_odom_.translation().y;
        t.transform.rotation = tf2::toMsg(q_correct);

        tf_broadcaster_->sendTransform(t);

        if (path_.poses.size() > 50) {
          path_.poses.erase(path_.poses.begin());
        }
        path_.poses.push_back(construct_path_msg());
        path_.header.stamp = time;
        path_publisher_->publish(path_);

      }
    }
  }

  geometry_msgs::msg::PoseStamped construct_path_msg()
  {
    geometry_msgs::msg::PoseStamped pose;
    pose.header.stamp = time;
    pose.header.frame_id = "nusim/world";
    pose.pose.position.x = xi_(1);
    pose.pose.position.y = xi_(2);

    tf2::Quaternion q_tf2;
    q_tf2.setRPY(0, 0, xi_(0));
    pose.pose.orientation = tf2::toMsg(q_tf2);

    return pose;
  }

  void initialize_xi()
  {
    xi_ = arma::join_vert(arma::zeros(3, 1), map_);
  }

  void initialize_Q()
  {
    Qbar_ = arma::eye(3, 3);
    Qbar_ *= Qfactor_;
    Qbar_ = arma::join_vert(Qbar_, arma::zeros(2 * n_, 3));
    Qbar_ = arma::join_horiz(Qbar_, arma::zeros(3 + 2 * n_, 2 * n_));
  }

  void initialize_sigma()
  {
    arma::mat sigma_q = arma::zeros(3, 3);
    arma::mat sigma_m = arma::eye(2 * n_, 2 * n_) * sigma_diag_;
    arma::mat sigma_qz = arma::join_vert(sigma_q, arma::zeros(2 * n_, 3));
    arma::mat sigma_mz = arma::join_vert(arma::zeros(3, 2 * n_), sigma_m);
    sigma_ = arma::join_horiz(sigma_qz, sigma_mz);
  }

  arma::mat calculate_A(turtlelib::Twist2D Vb)
  {
    // TODO: Clean this up
    arma::mat A;
    if (Vb.omega < 1e-3) {
      arma::mat tmp = {{0, 0, 0},
        {-Vb.x * std::sin(xi_(0)), 0, 0},
        {Vb.x * std::cos(xi_(0)), 0, 0}};
      tmp = arma::join_vert(tmp, arma::zeros(2 * n_, 3));
      A = arma::join_horiz(tmp, arma::zeros(3 + 2 * n_, 2 * n_));
      A = arma::eye(3 + 2 * n_, 3 + 2 * n_) + A;
    } else {
      arma::mat tmp = {{0, 0, 0},
        {-Vb.x / Vb.omega * std::cos(xi_(0)) + Vb.x / Vb.omega *
          std::cos(
            turtlelib::normalize_angle(
              xi_(0) + Vb.omega)), 0, 0},
        {-Vb.x / Vb.omega * std::sin(xi_(0)) + Vb.x / Vb.omega *
          std::sin(
            turtlelib::normalize_angle(
              xi_(0) + Vb.omega)), 0, 0}};
      tmp = arma::join_vert(tmp, arma::zeros(2 * n_, 3));
      A = arma::join_horiz(tmp, arma::zeros(3 + 2 * n_, 2 * n_));
      A = arma::eye(3 + 2 * n_, 3 + 2 * n_) + A;
    }

    return A;
  }

  arma::mat calc_H(double mx, double my, int j)
  {
    // RCLCPP_INFO_STREAM(get_logger(), "j in H: " << j);
    // RCLCPP_INFO_STREAM(get_logger(), "n in H: " << n_);
    // RCLCPP_INFO_STREAM(get_logger(), "map_: " << map_);
    double delta_x = mx - xi_(1);
    double delta_y = my - xi_(2);
    double d = std::pow(delta_x, 2) + std::pow(delta_y, 2);

    arma::mat block1{{0, -delta_x / std::sqrt(d), -delta_y / std::sqrt(d)},
      {-1, delta_y / d, -delta_x / d}};
    // RCLCPP_INFO_STREAM(get_logger(), "block1: " << block1);
    arma::mat block2 = arma::join_vert(arma::zeros(1, 2 * (j)), arma::zeros(1, 2 * (j)));
    // RCLCPP_INFO_STREAM(get_logger(), "block2: " << block2);
    arma::mat block3{{delta_x / std::sqrt(d), delta_y / std::sqrt(d)},
      {-delta_y / d, delta_x / d}};
    // RCLCPP_INFO_STREAM(get_logger(), "block3: " << block3);
    arma::mat block4 =
      arma::join_vert(arma::zeros(1, 2 * n_ - 2 * (j + 1)), arma::zeros(1, 2 * n_ - 2 * (j + 1)));
    // RCLCPP_INFO_STREAM(get_logger(), "block4: " << block4);

    arma::mat block12 = arma::join_horiz(block1, block2);
    arma::mat block34 = arma::join_horiz(block3, block4);
    arma::mat H = arma::join_horiz(block12, block34);

    return H;
  }

  arma::colvec get_range_bearing_measurement(double mx, double my)
  {
    arma::colvec z(2, arma::fill::zeros);
    z(0) = std::sqrt(std::pow(mx - xi_(1), 2) + std::pow(my - xi_(2), 2));
    z(1) = turtlelib::normalize_angle(std::atan2(my - xi_(2), mx - xi_(1)) - xi_(0));
    return z;
  }

  /// @brief When a new obstacle estimate is received, update the current, previous,
  /// and delta configuration variables. Also update the map, and update the H matrix.
  void fake_sensor_callback(const visualization_msgs::msg::MarkerArray & msg)
  {
    // RCLCPP_INFO_STREAM(get_logger(), "use_fake_sensors is: " << use_fake_sensors);
    if (use_fake_sensors) {
      std_msgs::msg::Header header;
      time = get_clock()->now();
      header.stamp = time;

      if (msg.markers.size() != map_.size()/2) {
        adjust_matrices(msg);
      }

      for (int j = 0; j < static_cast<int>(msg.markers.size()); j++) {
        // this is the location of the marker in the robot's frame
        turtlelib::Point2D m = map_to_robot_(turtlelib::Point2D{msg.markers.at(j).pose.position.x,
            msg.markers.at(j).pose.position.y});

        // update the measurement model
        arma::mat Hj = calc_H(m.x, m.y, j);

        // perform the SLAM update

        // calculate the Kalman gain
        arma::mat K = sigma_ * Hj.t() * arma::inv(Hj * sigma_ * Hj.t() + R_);

        // calculate the position of the robot based on current measurements
        arma::colvec z = get_range_bearing_measurement(m.x, m.y);

        // calculate the position of the robot based on previous measurements
        arma::colvec z_hat = get_range_bearing_measurement(map_(j*2), map_(j*2+1));

        arma::mat z_diff = arma::zeros(2, 1);
        z_diff(0) = z(0) - z_hat(0);
        z_diff(1) = turtlelib::normalize_angle(z(1) - z_hat(1));

        // calculate the correction to the robot's position
        correction_ = K * (z_diff);
        turtlelib::Transform2D map_to_odom_new{{correction_(1), correction_(2)}, correction_(0)};
        map_to_odom_ = map_to_odom_new * map_to_odom_;

        // don't think this matters at all, but ok!
        xi_ = xi_ + correction_;

        // compute the posterior covariance
        sigma_ = (arma::eye(xi_.size(), xi_.size()) - K * Hj) * sigma_;

        // why is the map drifting?
        xi_.rows(3,8) = map_;

        // correct the position of the robot in the map frame by updating
        // the transform from map to odom
        geometry_msgs::msg::TransformStamped t;
        t.header.stamp = time;
        t.header.frame_id = "map";
        t.child_frame_id = "green/odom";

        tf2::Quaternion q_correct;
        q_correct.setRPY(0, 0, map_to_odom_.rotation());

        t.transform.translation.x = map_to_odom_.translation().x;
        t.transform.translation.y = map_to_odom_.translation().y;
        t.transform.rotation = tf2::toMsg(q_correct);

        tf_broadcaster_->sendTransform(t);

        if (path_.poses.size() > 500) {
          path_.poses.erase(path_.poses.begin());
        }
        path_.poses.push_back(construct_path_msg());
        path_.header.stamp = time;
        path_publisher_->publish(path_);

      }
    }

  }

  void odometry_callback(const nav_msgs::msg::Odometry & msg)
  {
    tf2::Quaternion q(msg.pose.pose.orientation.x, msg.pose.pose.orientation.y,
      msg.pose.pose.orientation.z, msg.pose.pose.orientation.w);
    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    Vb_ = turtlelib::Twist2D{msg.twist.twist.linear.x,
                             msg.twist.twist.linear.y,
                             msg.twist.twist.angular.z};

    odom_to_robot_ = turtlelib::Transform2D{{msg.pose.pose.position.x, msg.pose.pose.position.y}, yaw};
    map_to_robot_ = map_to_odom_ * odom_to_robot_;
    xi_(0) = map_to_robot_.rotation();
    xi_(1) = map_to_robot_.translation().x;
    xi_(2) = map_to_robot_.translation().y;

    A_ = calculate_A(Vb_);
    sigma_ = A_ * sigma_ * A_.t() + Qbar_;
  }


  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_sub_;
  rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr fake_sensor_sub_;
  rclcpp::Subscription<nuslam::msg::Landmarks>::SharedPtr landmark_sub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_publisher_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr landmark_publisher_;

  // transform broadcaster
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  nav_msgs::msg::Path path_;

  arma::mat correction_;

  arma::mat map_;
  arma::mat xi_;
  arma::mat sigma_;
  arma::mat Qbar_;
  arma::mat A_;
  arma::mat H_;
  arma::mat R_;
  turtlelib::Twist2D Vb_;
  turtlelib::Twist2D Vb_minus_;
  turtlelib::Transform2D map_to_robot_; // map is always at (0,0) for us
  turtlelib::Transform2D odom_to_robot_;
  turtlelib::Transform2D map_to_odom_;
  double Qfactor_;
  double Rfactor_;
  double sigma_diag_;
  builtin_interfaces::msg::Time time;

  double n_;
  vector<double> map_ids_;

  bool use_fake_sensors;

  std::normal_distribution<> noise_generator_;


};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Slam>());
  rclcpp::shutdown();
  return 0;
}
