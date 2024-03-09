#include "turtlelib/geometry2d.hpp"
#include <iostream>

namespace turtlelib
{
double normalize_angle(double rad)
{
  while (rad > PI || rad <= -PI) {
    if (rad > PI) {
      rad -= 2*PI;
    } else if (rad <= -PI) {
      rad += 2*PI;
    }
  }
  return rad;
}

std::ostream & operator<<(std::ostream & os, const Point2D & p)
{
  os << "[" << p.x << " " << p.y << "]";

  return os;
}


std::istream & operator>>(std::istream & is, Point2D & p)
{
  if (is.peek() == '[') {
    is.ignore();
  }

  is >> p.x >> p.y;
  is.ignore();

  return is;
}

Vector2D operator-(const Point2D & head, const Point2D & tail)
{
  return {head.x - tail.x, head.y - tail.y};
}

Point2D operator+(const Point2D & tail, const Vector2D & disp)
{
  return {tail.x + disp.x, tail.y + disp.y};
}

Point2D operator*(Point2D lhs, const double & rhs)
{
  lhs.x *= rhs;
  lhs.y *= rhs;
  return lhs;
}

Vector2D & operator*=(Vector2D & lhs, const double & rhs)
{
  lhs.x *= rhs;
  lhs.y *= rhs;
  return lhs;
}

Vector2D operator*(Vector2D lhs, const double & rhs)
{
  return lhs *= rhs;
}

Vector2D & operator+=(Vector2D & lhs, Vector2D rhs)
{
  lhs.x += rhs.x;
  lhs.y += rhs.y;
  return lhs;
}

Vector2D operator+(Vector2D lhs, Vector2D rhs)
{
  lhs += rhs;
  return lhs;
}

Vector2D & operator-=(Vector2D & lhs, Vector2D rhs)
{
  lhs.x -= rhs.x;
  lhs.y -= rhs.y;
  return lhs;
}

Vector2D operator-(Vector2D lhs, Vector2D rhs)
{
  lhs -= rhs;
  return lhs;
}

double dot(Vector2D lhs, Vector2D rhs)
{
  return lhs.x * rhs.x + lhs.y * rhs.y;
}

double magnitude(Vector2D v)
{
  return std::sqrt(std::pow(v.x, 2) + std::pow(v.y, 2));
}

double angle(Vector2D lhs, Vector2D rhs)
{
  return std::acos(dot(lhs, rhs) / magnitude(lhs) / magnitude(rhs));

}

std::ostream & operator<<(std::ostream & os, const Vector2D & v)
{
  return os << "[" << v.x << " " << v.y << "]";
}

std::istream & operator>>(std::istream & is, Vector2D & v)
{
  if (is.peek() == '[') {
    is.ignore();
  }

  is >> v.x >> v.y;

  return is;
}

Vector2D normalize_vector(Vector2D v)
{
  return v * (1 / (magnitude(v)));
}


} // namespace turtlelib
