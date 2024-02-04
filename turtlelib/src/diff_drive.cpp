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

DiffDrive::DiffDrive(double length, double width, double radius)
{
  L = length;
  D = width;
  r = radius;
  H = construct_H_matrix();
  H_pseudo = construct_H_pseudo_matrix();
}

DiffDrive::DiffDrive(double x, double y, double theta)
{
  q = Configuration{x, y, theta};
  H = construct_H_matrix();
  H_pseudo = construct_H_pseudo_matrix();
}

DiffDrive::DiffDrive(double x, double y, double theta, double length, double width, double radius)
{
  q = Configuration{x, y, theta};
  L = length;
  D = width;
  r = radius;
  H = construct_H_matrix();
  H_pseudo = construct_H_pseudo_matrix();
}

void DiffDrive::FK(double phi_r_p, double phi_l_p)
{
  double d_phi_r = phi_r_p - phi_r;
  double d_phi_l = phi_r_p - phi_l;
  vector<double> u{d_phi_r, d_phi_l};
  vector<double> Vb_mat(vector<double>(2, 0.0));

  // calculate Vb based on the pseudo inverse of the H matrix
  // and the input controls
  for (int i = 0; i < (int)H_pseudo.size(); i++) {
    for (int j = 0; j < (int)H_pseudo[0].size(); j++) {
      Vb_mat[i] = H_pseudo[i][j] * u[j];
    }
  }

  // create the twist
  Twist2D Vb{Vb_mat[0], Vb_mat[1], Vb_mat[2]};

  // if Vb has zero angular displacement:
  Transform2D Tbb_prime;
  if (!Vb.omega) {
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
  q.theta += Vb.omega;
  q.x += Twb_prime.translation().x;
  q.y += Twb_prime.translation().y;
}

void DiffDrive::IK(Twist2D twist)
{
  vector<double> phi_delta(2, 0.0);
  vector<double> Vb = {twist.omega, twist.x, twist.y};

  double threshold = 1e10;
  if (twist.y >= threshold || twist.y <= -threshold) {
    throw std::logic_error("wheels are slipping!! Vb.y is not 0");
  }

  // multiple the H matrix by the body twist to find the new wheel velocities
  for (int i = 0; i < (int)phi_delta.size(); i++) {
    for (int j = 0; j < (int)phi_delta.size(); j++) {
      phi_delta[i] += H[i][j] * Vb[j];
    }
  }

}

vector<vector<double>> DiffDrive::construct_H_matrix()
{
  vector<vector<double>> H_tmp = {{-D, 1, 0},
    {D, 1, 0}};

  for (int i = 0; i < (int)H_tmp.size(); i++) {
    for (int j = 0; j < (int)H_tmp.size(); j++) {
      H_tmp[i][j] *= r; // r/3???
    }
  }

  return H_tmp;
}

vector<vector<double>> DiffDrive::construct_H_pseudo_matrix()
{
  vector<vector<double>> H_pseudo_tmp = {{-1 / D, 1, -1}, {1 / D, 1, 1}};

  for (int i = 0; i < (int)H_pseudo_tmp.size(); i++) {
    for (int j = 0; j < (int)H_pseudo_tmp.size(); j++) {
      H_pseudo_tmp[i][j] *= 1 / r;
    }
  }
}
}
