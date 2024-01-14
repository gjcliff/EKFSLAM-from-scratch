#include "turtlelib/geometry2d.hpp"
#include <iosfwd> // contains forward definitions for iostream objects
#include <iostream>
#include <vector>

namespace turtlelib
{
    double normalize_angle(double rad){
        while (rad > PI || rad < -PI){
            if (rad > PI){
                rad -= PI;
            } else if (rad < -PI){
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
            is >> p.x;
            is >> p.y;
            is.ignore();
        } else {
            is >> p.x >> p.y;
        }
        
        return is;
    }

    Vector2D operator-(const Point2D & head, const Point2D & tail){

    }

    Point2D operator+(const Point2D & tail, const Vector2D & disp){

    }

    std::ostream & operator<<(std::ostream & os, const Vector2D & v){

    }

    std::istream & operator>>(std::istream & is, Vector2D & v){

    }

} // namespace turtlelib
