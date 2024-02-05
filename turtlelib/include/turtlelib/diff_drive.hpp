#ifndef TURTLELIB_DIFFDRIVE_HPP_INCLUDE_GUARD
#define TURTLELIB_DIFFDRIVE_HPP_INCLUDE_GUARD
/// @file
/// @brief Model the kinematics of a differential drive robot.


#include "turtlelib/geometry2d.hpp"
#include "turtlelib/se2d.hpp"
#include <vector>

namespace turtlelib
{

/// @brief a robot configuration in the world frame
struct Configuration
{
  /// @brief the x coordinate
  double x = 0.0;

  /// @brief the y coordinate
  double y = 0.0;

  /// @brief the angle
  double theta = 0.0;
};

struct RobotDimensions
{
  double L = 1.0;
  double D = 1.0;
  double r = 0.1;
};

class DiffDrive
{
public:
  /// @brief create a model of the kinematics of a differential drive robot
  DiffDrive();

  /// @brief create a model of the kinematics of a differential drive robot
  /// that specifies the initial positions of the robot's wheels
  /// @param phi_left - the position of the robot's left wheel
  /// @param phi_right - the position of the robot's right wheel
  DiffDrive(double phi_left, double phi_right);

  /// @brief create the model of the kinematics of a differential drive robot
  /// that specifies the positions of the robot's wheels in relation to is
  /// bodyf frame at the center of the robot.
  /// @param rd - the dimensions of the robot including x and y positions of the
  /// robot's wheels from it's center, and the wheel radius
  DiffDrive(RobotDimensions rd);

  /// @brief create a model of the kinematics of a differential drive robot
  /// that specifies the initial configuration of the robot
  /// @param q_orig - the original configuration of the robot
  DiffDrive(Configuration q_orig);

  /// @brief create a model of the kinematics of a differential drive robot
  /// that specifies the initial configuration of the robot and additionally
  /// the positions of the robot's wheels in relation to its body frame at the
  /// center of the robot
  /// @param q_orig - the original configuration of the robot
  /// @param rd - the dimensions of the robot
  DiffDrive(Configuration q_orig, RobotDimensions rd);

  /// @brief given new wheel positions, update the configuration
  /// @param phi_r_p
  /// @param phi_l_p
  void FK(double phi_l_p, double phi_r_p);

  /// @brief compute the wheel velocities required to make the robot move
  /// at a given body twist
  /// @param twist
  void IK(Twist2D twist);

  /// @brief retrieve the robot's current configuration
  /// @return the robot's current configuration
  Configuration get_current_configuration();

  /// @brief retrieve the robot's current dimensions
  /// @return the robot's current dimensions
  RobotDimensions get_robot_dimensions();

  /// @brief construct the H matrix
  /// @return a 2x3 H matrix
  vector<vector<double>> construct_H_matrix();

  /// @brief construct the pseudo H matrix
  /// @return a 3x2 pseudo H matrix
  vector<vector<double>> construct_H_pseudo_matrix();

private:
  double phi_l = 0.0;
  double phi_r = 0.0;
  RobotDimensions rd;
  Configuration q;
  Transform2D Tb1 = Transform2D({rd.L, rd.D}, 0.0);
  Transform2D Tb2 = Transform2D({rd.L, -rd.D}, 0.0);
  vector<vector<double>> H;
  vector<vector<double>> H_pseudo;
};
}

#endif
