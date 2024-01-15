#include "turtlelib/se2d.hpp"
#include "turtlelib/geometry2d.hpp"
#include <iosfwd>
#include <iostream>
#include <vector>
#include <cmath>

using std::vector;
namespace turtlelib
{

    std::ostream & operator<<(std::ostream & os, const Twist2D & tw){
        os << "[" << tw.omega << " " << tw.x << " " << tw.y << "]";
        return os;
    }

    std::istream & operator>>(std::istream & is, Twist2D & tw){
        if (is.peek() == '['){
            is.ignore();
        }

        is >> tw.omega >> tw.x >> tw.y;
        return is;
    }

    Transform2D::Transform2D(){
        t = {{std::cos(rad),-std::sin(rad),x},
             {std::sin(rad),std::cos(rad),y},
             {0,0,1}};
    }

    Transform2D::Transform2D(Vector2D trans){
        x = trans.x;
        y = trans.y;
        t = {{std::cos(rad),-std::sin(rad),x},
             {std::sin(rad),std::cos(rad),y},
             {0,0,1}};
    }

    Transform2D::Transform2D(double radians){
        rad = radians;
        t = {{std::cos(rad),-std::sin(rad),x},
             {std::sin(rad),std::cos(rad),y},
             {0,0,1}};
    }



    Transform2D::Transform2D(Vector2D trans, double radians){
        rad = radians;
        x = trans.x;
        y = trans.y;
        t = {{std::cos(rad),-std::sin(rad),x},
             {std::sin(rad),std::cos(rad),y},
             {0,0,1}};
    }

    Point2D Transform2D::operator()(Point2D p) const{
        vector<double> mat_p = {p.x, p.y, 1}; // create a tempoarary 3x1 vector so we can do the math.
        vector<double> output(3,0); // create a vector that we'll construct a Point2D object from after the multiplication is done.

        for (unsigned int i = 0; i < t[0].size(); i ++){
            for (unsigned int j = 0; j < t.size(); j ++){
                output[i] += t[i][j] * mat_p[j];
            }
        }

        Point2D out_p;
        out_p = {output[0], output[1]};

        return out_p;
    }


    Vector2D Transform2D::operator()(Vector2D v) const{
        vector<vector<double>> tmp_t = t;
        tmp_t[0][2] = 0; // set the x translation portion of the transformatoin matrix equal to 0
        tmp_t[1][2] = 0; // set the y translation portion of the transformation matrix equal to 0

        vector<double> mat_v = {v.x, v.y, 1};
        vector<double> output(3,0);

        for (unsigned int i = 0; i < tmp_t[0].size(); i++){
            for (unsigned int j = 0; j < tmp_t.size(); j++){
                output[i] += t[i][j] * mat_v[j];
            }
        }

        Vector2D out_v;
        out_v = {output[0], output[1]};

        return out_v;
    }


    Twist2D Transform2D::operator()(Twist2D v) const{
        vector<double> mat_v = {v.omega, v.x, v.y};
        vector<double> output(3,0);

        for (unsigned int i = 0; i < t[0].size(); i++){
            for (unsigned int j = 0; j < t.size(); j++){
                output[i] += t[i][j] * mat_v[j];
            }
        }

        Twist2D out_v;
        out_v = {output[0], output[1], output[2]};

        return out_v;
    }

    // Transform2D Transform2D::inv() const{

    // }

    // Transform2D & Transform2D::operator*=(const Transform2D & rhs){

    // }

    // Vector2D Transform2D::translation() const{

    // }

    // double Transform2D::rotation() const{

    // }

    // std::ostream & operator<<(std::ostream & os, const Transform2D & tf){

    // }

    // std::istream & operator>>(std::istream & is, Transform2D & tf){

    // }

    // Transform2D operator*(Transform2D lhs, const Transform2D & rhs){

    // }
}