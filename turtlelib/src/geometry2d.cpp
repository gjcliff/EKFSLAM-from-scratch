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
        
        return is;
    }

    Vector2D operator-(const Point2D & head, const Point2D & tail){
        Vector2D vec;
        vec.x = head.x - tail.x;
        vec.y = head.y - tail.y;

        return vec;
    }

    Point2D operator+(const Point2D & tail, const Vector2D & disp){
        Point2D head;
        head.x = tail.x + disp.x;
        head.y = tail.y + disp.y;
        
        return head;

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
        
        return is;
    }

} // namespace turtlelib
