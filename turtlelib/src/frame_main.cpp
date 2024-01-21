#include"turtlelib/geometry2d.hpp"
#include"turtlelib/se2d.hpp"
#include"turtlelib/svg.hpp"
#include<iostream>

using turtlelib::Svg, turtlelib::Transform2D, turtlelib::Point2D, turtlelib::Vector2D, turtlelib::Twist2D;

int main()
{
    Transform2D T_ab;
    Transform2D T_bc;
    Svg svg("/tmp/frames.svg");

    // I need to display each of the transforms w.r.t. the {a} frame.
    // Theoretically, the locations of frames {a}, {b}, and {c} shouldn't
    // change w.r.t. the world. The {a} frame in the {b} frame should be in
    // the same location as the {a} frame in the {c} frame. It's still the
    // {a} frame, and it's absolute position on the page shouldn't change.

    // TRANSFORMS
    // capture T_{a,b}
    std::cout << "Enter the transform T_{a,b}:" << std::endl;
    std::cin >> T_ab;

    // capture T_{b,c}
    std::cout << "Enter the transform T_{b,c}:" << std::endl;
    std::cin >> T_bc;

    // compute T_{a,b}
    std::cout << "T_{a,b}: " << T_ab << std::endl;
    svg.draw_coordiante_frame(T_ab, "T_{a,b}");

    // compute T_{b,a}
    Transform2D T_ba = T_ab.inv();
    std::cout << "T_{b,a}: " << T_ba << std::endl;
    svg.draw_coordiante_frame(T_ab * T_ba, "T_{b,a}");

    // compute T_{b,c}
    std::cout << "T_{b,c}: " << T_bc << std::endl;
    svg.draw_coordiante_frame(T_ab * T_bc, "T_{b,c}");

    // compute T_{c,b}
    Transform2D T_cb = T_bc.inv();
    std::cout << "T_{c,b}: " << T_cb << std::endl;
    svg.draw_coordiante_frame(T_ab * T_bc * T_cb, "T_{c,b}");

    // compute T_{a,c}
    Transform2D T_ac = T_ab * T_bc;
    std::cout << "T_{a,c}: " << T_ac << std::endl;
    svg.draw_coordiante_frame(T_ac, "T_{a,c}");

    //compute T_{c,a}
    Transform2D T_ca = T_ac.inv();
    std::cout << "T_{c,a}: " << T_ca << std::endl;
    svg.draw_coordiante_frame(T_ac * T_ca, "T_{c,a}");

    // POINTS
    // retrieve the point p_a
    Point2D p_a;
    std::cout << "Enter point p_a:" << std::endl;
    std::cin >> p_a;

    // compute p_a
    std::cout << "p_a: " << p_a << std::endl;
    svg.draw_point(p_a, "purple");

    // compute p_b
    Point2D p_b;
    p_b = T_ba(p_a);
    std::cout << "p_b: " << p_b << std::endl;
    svg.draw_point(p_b, "brown");

    // compute p_c
    Point2D p_c;
    p_c = T_ca(p_a);
    std::cout << "p_c: " << p_c << std::endl;
    svg.draw_point(p_c, "orange");

    // VECTORS
    // retrieve vector v_b
    Vector2D v_b;
    std::cout << "enter vector v_b: " << std::endl;
    std::cin >> v_b;

    // compute v_bhat
    std::cout << "v_bhat: " << normalize_vector(v_b) << std::endl;
    svg.draw_vector(normalize_vector(v_b), "brown");

    // compute v_a
    std::cout << "v_a: " << T_ab(v_b) << std::endl;
    svg.draw_vector(T_ab(v_b), "purple");

    // display v_b
    std::cout << "v_b: " << v_b << std::endl;

    // compute v_c
    std::cout << "v_c: " << T_cb(v_b) << std::endl;
    Point2D tail = {T_ac.translation().x, T_ac.translation().y};
    svg.draw_vector(T_ac(T_cb(v_b)), tail, "orange");

    // TWISTS
    // retrieve twist V_b
    Twist2D V_b;
    std::cout << "Enter twist V_b:" << std::endl;
    std::cin >> V_b;

    // compute V_a
    std::cout << "V_a: " << T_ab(V_b) << std::endl;

    // display V_b
    std::cout << "V_b: " << V_b << std::endl;

    // compute V_c
    std::cout << "V_c: " << T_cb(V_b) << std::endl;
    
    svg.close();
    return 0;
}


