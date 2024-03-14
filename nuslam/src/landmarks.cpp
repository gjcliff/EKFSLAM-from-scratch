#include <chrono>
#include <functional>
#include <memory>
#include <rclcpp/logging.hpp>
#include <string>
#include <turtlelib/geometry2d.hpp>
#include <vector>
#include <cmath>
#include <armadillo>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "turtlelib/geometry2d.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "nuslam/msg/circle.hpp"
#include "nuslam/msg/landmarks.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */

class Landmarks : public rclcpp::Node
{
  public:
    Landmarks()
    : Node("landmarks")
    {
      scan_subscriber_ = create_subscription<sensor_msgs::msg::LaserScan>(
          "scan", 10, std::bind(&Landmarks::scan_callback, this, _1));
      landmarks_publisher = create_publisher<nuslam::msg::Landmarks>("landmarks", 10);
      tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
      tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
      tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    }

  private:
    geometry_msgs::msg::TransformStamped get_transform(
      std::string fromFrameRel,
      std::string toFrameRel) const
    {
      geometry_msgs::msg::TransformStamped t;

      // Look up for the transformation between target_frame and turtle2 frames
      // and send velocity commands for turtle2 to reach target_frame
      try {
        t = tf_buffer_->lookupTransform(
          fromFrameRel, toFrameRel,
          tf2::TimePointZero);
        return t;
      } catch (const tf2::TransformException & ex) {
        RCLCPP_INFO(
          this->get_logger(), "Could not transform %s to %s: %s",
          toFrameRel.c_str(), fromFrameRel.c_str(), ex.what());
        return t;
      }
    }

    std::vector<std::vector<double>> find_clusters(const sensor_msgs::msg::LaserScan & msg) const
    {
      std::vector<std::vector<double>> clusters;
      for (unsigned int i = 0; i < msg.ranges.size(); i++) {
        // if (msg.ranges.at(i) > -1e-5 && msg.ranges.at(i) < 1e-5) {
        //   continue;
        // }
        if (clusters.size() == 0) {
          clusters.push_back({msg.ranges.at(i) * std::cos(i * msg.angle_increment),
                              msg.ranges.at(i) * std::sin(i * msg.angle_increment)});
        } else if (i == msg.ranges.size()-1) {
          if (msg.ranges.at(i) - msg.ranges.at(i-1) < 0.05) {
            clusters.back().push_back(msg.ranges.at(i) * std::cos(i * msg.angle_increment));
            clusters.back().push_back(msg.ranges.at(i) * std::sin(i * msg.angle_increment));
          } else {
            if (clusters.back().size() < 8) {
              clusters.erase(clusters.end()-1);
            }
          clusters.push_back({msg.ranges.at(i) * std::cos(i * msg.angle_increment),
                              msg.ranges.at(i) * std::sin(i * msg.angle_increment)});
          }
          // check if we need to combine the first and last clusters
          if (msg.ranges.at(i) - msg.ranges.at(0) < 0.05) {
            // BEGIN CITATION [32]
            clusters.at(clusters.size()-1).insert(clusters.at(clusters.size()-1).end(), clusters.at(0).begin(), clusters.at(0).end());
            // END CITATION [32]
          } else {
            if (clusters.back().size() < 8) {
              clusters.erase(clusters.end()-1);
            }
          }

        } else {
          if (std::abs(msg.ranges.at(i) - msg.ranges[i-1]) < 0.05) {
            // BEGIN CITATION [31]
            clusters.back().push_back(msg.ranges.at(i) * std::cos(i * msg.angle_increment));
            clusters.back().push_back(msg.ranges.at(i) * std::sin(i * msg.angle_increment));
            // END CITATION [31]
          } else {
            if (clusters.back().size() < 8) {
              clusters.erase(clusters.end()-1);
            }
          clusters.push_back({msg.ranges.at(i) * std::cos(i * msg.angle_increment),
                              msg.ranges.at(i) * std::sin(i * msg.angle_increment)});
          }
        }
      }
      for (unsigned int i = 0; i < clusters.size(); i ++){
        // for (unsigned int j = 0; j < clusters.at(i).size(); j++) {
        //   RCLCPP_INFO_STREAM(get_logger(), "cluster " << i << ": " << clusters.at(i).at(j));
        // }
        // RCLCPP_INFO_STREAM(get_logger(), "cluser size: " << clusters.at(i).size());
      }
      // RCLCPP_INFO_STREAM(get_logger(), "");
      return clusters;
    }

    void fit_circles(std::vector<std::vector<double>> clusters) const
    {
      // find the centroid of the laser scan cluseters
      nuslam::msg::Landmarks landmarks;
      for (unsigned int i = 0; i < clusters.size(); i++) {
        int n = clusters.at(i).size()/2;
        double x_hat = 0;
        double y_hat = 0;
        // find the centroid of the cluster
        for (int j = 0; j < n; j++) {
          x_hat += clusters.at(i).at(j*2)/n;
          y_hat += clusters.at(i).at(j*2+1)/n;
        }
        // shift the coordinates so that the centroid is at the origin
        std::vector<double> shifted_cluster = clusters.at(i);
        double z_bar = 0;
        for (int j = 0; j < n; j++) {
          shifted_cluster.at(j*2) -= x_hat;
          shifted_cluster.at(j*2+1) -= y_hat;
          z_bar += (std::pow(shifted_cluster.at(j*2),2) + std::pow(shifted_cluster.at(j*2+1),2))/(shifted_cluster.size()/2.0);
        }
        // form the data matrix from the n data points
        arma::mat Z(shifted_cluster.size()/2.0, 4, arma::fill::zeros);
        for (int j = 0; j < n; j++) {
          arma::mat tmp = {std::pow(shifted_cluster.at(j*2),2) + std::pow(shifted_cluster.at(j*2+1),2), shifted_cluster.at(j*2), shifted_cluster.at(j*2+1), 1};
          Z.row(j) = tmp;
        }
        // form the moment matrix
        arma::mat M = (1.0/n) * Z.t() * Z;
        // form the constraint matrix for the "hyperaccurate algebraic fit", huh?
        arma::mat H = {{8*z_bar, 0, 0, 2}, {0, 1, 0, 0},{0, 0, 1, 0}, {2, 0, 0, 0}};
        // compute the inverse of H
        arma::mat H_inv = {{0, 0, 0, 0.5}, {0, 1, 0, 0}, {0, 0, 1, 0}, {0.5, 0, 0, -2 * z_bar}};
        // compute the singular value decomposition of Z
        arma::mat U;
        arma::vec s_vec;
        arma::mat V;
        arma::svd(U,s_vec,V,Z);
        arma::mat s = arma::diagmat(s_vec);
        // if the smallest singular value of the svd is less that 10e-12...
        arma::mat A;
        if (arma::min(s_vec) < 10e-12) {
          A = V.col(3);
        } else {
          arma::mat Y = V.t() * s * V;
          arma::mat Q = Y * H_inv * Y;
          // then find the eigen values and eigen vectors of Q
          arma::vec eigvals;
          arma::mat eigvecs;
          arma::eig_sym(eigvals, eigvecs, Q);
          arma::vec A_star = eigvecs.col(arma::index_min(eigvals));
          A = arma::inv(Y) * A_star;
        }
        double a = -A(1)/(2*A(0));
        double b = -A(2)/(2*A(0));
        double R = std::sqrt((std::pow(A(1),2) + std::pow(A(2),2) - 4*A(0)*A(3))/(4*std::pow(A(0),2)));

        double x_coord = a + x_hat;
        double y_coord = b + y_hat;

        // now we will determine if this cluster is really a circle or not using
        // law of cosines and the inscribed angle theorem
        std::vector<double> angles;
        double mean = 0;
        double stdev = 0;
        for (int j = 1; j < n-1; j++) {
          double c = turtlelib::magnitude({shifted_cluster.at(0) - shifted_cluster.at(n*2-2), shifted_cluster.at(1) - shifted_cluster.at(n*2-1)});
          double a = turtlelib::magnitude({shifted_cluster.at(0) - shifted_cluster.at(j*2), shifted_cluster.at(1) - shifted_cluster.at(j*2+1)});
          double b = turtlelib::magnitude({shifted_cluster.at(n*2-2) - shifted_cluster.at(j*2), shifted_cluster.at(n*2-1) - shifted_cluster.at(j*2+1)});

          double C = std::acos((std::pow(a,2) + std::pow(b,2) - std::pow(c,2))/(2*a*b));
          angles.push_back(C);
          mean += C/(n-2);
        }
        for (int j = 0; j < n-2; j++) {
          stdev += std::pow(angles.at(j) - mean, 2)/(n-2);
        }
        stdev = std::sqrt(stdev);

        if (stdev < 0.15 && turtlelib::deg2rad(90.0) < mean && mean < turtlelib::deg2rad(135.0)) {
          // RCLCPP_INFO_STREAM(get_logger(), "circle found at: " << x_coord << " " << y_coord);
          // RCLCPP_INFO_STREAM(get_logger(), "circle radius: " << R);
          // RCLCPP_INFO_STREAM(get_logger(), "circle stdev: " << stdev);
          // RCLCPP_INFO_STREAM(get_logger(), "circle mean: " << mean);
          // RCLCPP_INFO_STREAM(get_logger(), "circle n: " << n);
          nuslam::msg::Circle circle;
          circle.x = x_coord;
          circle.y = y_coord;
          circle.radius = R;
          landmarks.landmarks.push_back(circle);
        }
      }
      landmarks_publisher->publish(landmarks);
    }

    void scan_callback(const sensor_msgs::msg::LaserScan & msg) const
    {
      // get a scan, and start with the first point. As long as the next point
      // is within a certain distance threshold, add it to the current cluster.
      // As a final step, check if the final point is close to the first point,
      // and if it is then combine the final cluster and the first cluster.
      // 
      // discard any clusters with fewer than 3 points
      // I think here is where we would want to calculate the mahalanobis distance
      // later, in L.3. I think in this is just figuring out if something is
      // a landmakr or not, and not associating it with previous measuremnets
      
      std::vector<std::vector<double>> clusters = find_clusters(msg);
      fit_circles(clusters);
      // RCLCPP_INFO_STREAM(get_logger(), "");

    }
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscriber_;
    rclcpp::Publisher<nuslam::msg::Landmarks>::SharedPtr landmarks_publisher;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Landmarks>());
  rclcpp::shutdown();
  return 0;
}
