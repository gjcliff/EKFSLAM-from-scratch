#include "turtlelib/geometry2d.hpp"
#include <iosfwd> // contains forward definitions for iostream objects
#include <iostream>

namespace turtlelib
{
    double normalize_angle(double rad){
        while (rad > PI || rad <= -PI){
            if (rad > PI){
                rad -= PI;
            } else if (rad <= -PI){
                rad += PI;
            }
        }
        return rad;
    }

    std::ostream & operator<<(std::ostream & os, const Point2D & p){
        os << "["<< p.x << " " << p.y << "]";
        
        return os;
    }

    
    std::istream & operator>>(std::istream & is, Point2D & p){
        if (is.peek() == '['){
            is.ignore();
        } 

        is >> p.x >> p.y;
        // does not eat the last ']'
        return is;
    }

    Vector2D operator-(const Point2D & head, const Point2D & tail){
        Vector2D vec;
        vec.x = head.x - tail.x;
        vec.y = head.y - tail.y;

        return vec; // can return {head.x - tail.x, head.y - tail.y}
    }

    Point2D operator+(const Point2D & tail, const Vector2D & disp){
        Point2D head;
        head.x = tail.x + disp.x;
        head.y = tail.y + disp.y;
        
        return head; // return {} no temp needed

    }

    std::ostream & operator<<(std::ostream & os, const Vector2D & v){
        os << "[" << v.x << " " << v.y << "]";
        return os;
    }

    std::istream & operator>>(std::istream & is, Vector2D & v){
        if(is.peek() == '['){
            is.ignore();
        }

        is >> v.x >> v.y;
        // does not eat the last
        return is;
    }

    Vector2D normalize_vector(Vector2D v)
    {
        // do not * 1/x instead just / x
        return v * (1/(std::sqrt(std::pow(v.x, 2) + std::pow(v.y, 2))));
    }

    Point2D operator*(Point2D lhs, const double & rhs)
    {
        lhs.x *= rhs;
        lhs.y *= rhs;
        return lhs;
    }

    Vector2D operator*(Vector2D lhs, const double & rhs)
    {
        lhs.x *= rhs;
        lhs.y *= rhs;
        return lhs;
    }

} // namespace turtlelib
