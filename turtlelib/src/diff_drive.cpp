#include "turtlelib/geometry2d.hpp"
#include "turtlelib/se2d.hpp"
#include "turtlelib/diff_drive.hpp"
#include <vector>

namespace turtlelib
{

DiffDrive::DiffDrive()
{
  H = construct_H_matrix();
  H_pseudo = construct_H_pseudo_matrix();
}

DiffDrive::DiffDrive(double phi_right, double phi_left)
{
  phi_r = phi_right;
  phi_l = phi_left;
  H = construct_H_matrix();
  H_pseudo = construct_H_pseudo_matrix();
}

DiffDrive::DiffDrive(RobotDimensions rd)
{
  rd.L = rd.L;
  rd.D = rd.D;
  rd.r = rd.r;
  H = construct_H_matrix();
  H_pseudo = construct_H_pseudo_matrix();
}

DiffDrive::DiffDrive(Configuration q_orig)
{
  q = q_orig;
  H = construct_H_matrix();
  H_pseudo = construct_H_pseudo_matrix();
}

DiffDrive::DiffDrive(Configuration q_orig, RobotDimensions rd)
{
  q = q_orig;
  rd.L = rd.L;
  rd.D = rd.D;
  rd.r = rd.r;
  H = construct_H_matrix();
  H_pseudo = construct_H_pseudo_matrix();
}

Configuration DiffDrive::FK(double phi_l_p, double phi_r_p)
{
  double d_phi_r = phi_r_p - phi_r;
  double d_phi_l = phi_l_p - phi_l;
  vector<double> u{d_phi_l, d_phi_r};
  vector<double> Vb_mat(vector<double>(3, 0.0));

  // calculate Vb based on the pseudo inverse of the H matrix
  // and the input controls
  for (int i = 0; i < (int)H_pseudo.size(); i++) {
    for (int j = 0; j < (int)H_pseudo[0].size(); j++) {
      Vb_mat[i] += H_pseudo[i][j] * u[j];
    }
  }

  // create the twist
  Twist2D Vb{Vb_mat[0], Vb_mat[1], Vb_mat[2]};

  // if Vb has zero angular displacement:
  Transform2D Tbb_prime;
  if (Vb.omega <= 1e-5 && Vb.omega >= -1e-5) {
    // Integrate the twist to find Tbb_prime
    Tbb_prime = integrate_twist(Vb);
  } else { // else if Vb has angular displacement:
    // Find the center of rotation in a frame {s} using the adjoint. This
    // is represented by the transform Tsb from the body frame to the center
    // of rotation frame.
    double xs = Vb.y / Vb.omega;
    double ys = -Vb.x / Vb.omega;
    Transform2D Tsb({xs, ys});

    // find the translation representing the pure rotation in the new frame, {s}
    Transform2D Tss_prime(Vb.omega);

    // now translate back to the body frame location while keeping the new
    // orientaion
    Tbb_prime = Tsb.inv() * Tss_prime * Tsb;
  }

  // now, compute the transformation matrix of the robot body frame {b} in
  // the world frame {w}
  Transform2D Twb({q.x, q.y}, q.theta);
  Transform2D Twb_prime = Twb * Tbb_prime;
  q.theta += Twb_prime.rotation();
  q.x += Twb_prime.translation().x;
  q.y += Twb_prime.translation().y;

  return q;
}

vector<double> DiffDrive::IK(Twist2D twist)
{
  vector<double> phi_delta(2, 0.0);
  vector<double> Vb = {twist.omega, twist.x, twist.y};

  double threshold = 1e-10;
  if (twist.y >= threshold || twist.y <= -threshold) {
    throw std::logic_error("wheels are slipping!! Vb.y is not 0");
  }

  // multiple the H matrix by the body twist to find the new wheel velocities
  for (int i = 0; i < (int)H.size(); i++) {
    for (int j = 0; j < (int)H[0].size(); j++) {
      phi_delta[i] += H[i][j] * Vb[j];
    }
  }

  return phi_delta;
}

Configuration DiffDrive::get_current_configuration()
{
  return q;
}

RobotDimensions DiffDrive::get_robot_dimensions()
{
  return rd;
}

vector<vector<double>> DiffDrive::construct_H_matrix()
{
  vector<vector<double>> H_tmp = {{-rd.D, 1, 0},
    {rd.D, 1, 0}};

  for (int i = 0; i < (int)H_tmp.size(); i++) {
    for (int j = 0; j < (int)H_tmp[0].size(); j++) {
      H_tmp[i][j] *= 1 / rd.r; // r/3???
    }
  }

  return H_tmp;
}

vector<vector<double>> DiffDrive::construct_H_pseudo_matrix()
{
  // I calculated this using numpy, and am simply constructing the matrix here
  // in lieu of calculating the pseudo matrix by hand in C++ with no help from
  // libraries.
  vector<vector<double>> H_pseudo_tmp = {{-1 / rd.D, 1 / rd.D}, {1, 1}, {0, 0}};

  for (int i = 0; i < (int)H_pseudo_tmp.size(); i++) {
    for (int j = 0; j < (int)H_pseudo_tmp[0].size(); j++) {
      H_pseudo_tmp[i][j] *= rd.r / 2;
    }
  }

  return H_pseudo_tmp;
}
}
