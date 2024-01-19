#include "turtlelib/se2d.hpp"
#include "turtlelib/geometry2d.hpp"
#include <iosfwd>
#include <iostream>
#include <vector>
#include <cmath>
#include <iostream>

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
             {0.0,0.0,1.0}};
    }

    Transform2D::Transform2D(Vector2D trans){
        x = trans.x;
        y = trans.y;
        t = {{std::cos(rad),-std::sin(rad),x},
             {std::sin(rad),std::cos(rad),y},
             {0.0,0.0,1.0}};
    }

    Transform2D::Transform2D(double radians){
        rad = radians;
        t = {{std::cos(rad),-std::sin(rad),x},
             {std::sin(rad),std::cos(rad),y},
             {0.0,0.0,1.0}};
    }



    Transform2D::Transform2D(Vector2D trans, double radians){
        rad = radians;
        x = trans.x;
        y = trans.y;
        t = {{std::cos(rad),-std::sin(rad),x},
             {std::sin(rad),std::cos(rad),y},
             {0.0,0.0,1.0}};
    }

    Point2D Transform2D::operator()(Point2D p) const{
        vector<double> mat_p = {p.x, p.y, 1}; // create a tempoarary 3x1 vector so we can do the math.
        vector<double> output(3.0,0.0); // create a vector that we'll construct a Point2D object from after the multiplication is done.

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
        tmp_t[0][2] = 0.0; // set the x translation portion of the transformatoin matrix equal to 0
        tmp_t[1][2] = 0.0; // set the y translation portion of the transformation matrix equal to 0
        // this might not be the right way to do this
        vector<double> mat_v = {v.x, v.y, 1.0};
        vector<double> output(3.0,0.0);

        for (unsigned int i = 0; i < tmp_t.size(); i++){
            for (unsigned int j = 0; j < tmp_t[0].size(); j++){
                output[i] += t[i][j] * mat_v[j];
            }
        }

        Vector2D out_v;
        out_v = {output[0], output[1]};

        return out_v;
    }


    Twist2D Transform2D::operator()(Twist2D v) const{
        vector<double> mat_v = {v.omega, v.x, v.y};
        vector<double> output(3.0,0.0);

        for (unsigned int i = 0; i < t.size(); i++){
            for (unsigned int j = 0; j < t[0].size(); j++){
                output[i] += t[i][j] * mat_v[j];
            }
        }

        Twist2D out_v;
        out_v = {output[0], output[1], output[2]};

        return out_v;
    }

    Transform2D Transform2D::inv() const{
        // using the formula from ME449 the inverse of a transformation matrix
        vector<vector<double>> R_T = {{t[0][0], t[1][0]}, {t[0][1], t[1][1]}};
        vector<double> neg_R_TP(2.0,0.0);

        for (unsigned int i = 0; i < R_T.size(); i++){
            for (unsigned int j = 0; j < R_T[0].size(); j++){
                neg_R_TP[i] += -(R_T[i][j] * t[j][2]);
            }
        }

        Transform2D t_T = *this;
        t_T.t[0][0] = R_T[0][0];
        t_T.t[0][1] = R_T[0][1];
        t_T.t[1][0] = R_T[1][0];
        t_T.t[1][1] = R_T[1][1];
        t_T.t[0][2] = neg_R_TP[0];
        t_T.t[1][2] = neg_R_TP[1];

        return t_T;

    }

    Transform2D & Transform2D::operator*=(const Transform2D & rhs){
        vector<vector<double>> output(3, vector<double>(3,0));

        for (unsigned int i = 0; i < t.size(); i++){
            for (unsigned int j = 0; j < t.size(); j++){
                for (unsigned int k = 0; k < t.size(); k++){
                    output[i][j] += t[i][k] * rhs.t[k][j];
                }
            }
        }

        t = output;

        rad = std::acos(t[0][0]);
        x = t[0][2];
        y = t[1][2];

        

        return *this;
    }

    Vector2D Transform2D::translation() const{
        Vector2D trans = {x, y};
        return trans;
    }

    double Transform2D::rotation() const{
        return rad;
    }

    std::ostream & operator<<(std::ostream & os, const Transform2D & tf){
        os << "deg: " << tf.rotation() << " x: " << tf.translation().x << " y: " << tf.translation().y << std::endl;
        os << "[" << tf.t[0][0] << " " << tf.t[0][1] << " " <<tf.t[0][2] << "]" << std::endl;
        os << "[" << tf.t[1][0] << " " << tf.t[1][1] << " " <<tf.t[1][2] << "]" << std::endl;
        os << "[" << tf.t[2][0] << " " << tf.t[2][1] << " " <<tf.t[2][2] << "]" << std::endl;
        return os;
    }

    std::istream & operator>>(std::istream & is, Transform2D & tf){
        double radians;
        Vector2D trans;

        is >> radians >> trans.x >> trans.y;

        Transform2D temp(trans, radians);
        tf = temp;

        return is;
    }

    Transform2D operator*(Transform2D lhs, const Transform2D & rhs){
        return lhs*=rhs;
    }
}