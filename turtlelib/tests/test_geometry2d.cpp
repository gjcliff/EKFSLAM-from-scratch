#include "turtlelib/geometry2d.hpp"
#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include <iosfwd>
#include <iostream>
#include <sstream>


namespace turtlelib{
    TEST_CASE("almost equal works", "[almost_equal]")
    {
        double d1 = 1.0001;
        double d2 = 1.0001;
        double epsilon = 0.001;
        REQUIRE(almost_equal(d1, d2, epsilon));
    }

    TEST_CASE("deg2rad works", "[deg2rad]"){
        REQUIRE_THAT(deg2rad(90.0), Catch::Matchers::WithinAbs(1.5708, 0.001));
    }

    TEST_CASE("rad2deg works", "[rad2deg]"){
        REQUIRE_THAT(rad2deg(1.5708), Catch::Matchers::WithinAbs(90.0, 0.001));
    }

    TEST_CASE("normalize_angle works", "[normalize_angle]") {
        REQUIRE(normalize_angle(PI) == PI);
        REQUIRE(normalize_angle(-PI) == 0);
        REQUIRE(normalize_angle(0) == 0);
        REQUIRE(normalize_angle(-PI/4) ==  -PI/4);
        REQUIRE(normalize_angle(3*PI/2) == PI/2);
        REQUIRE(normalize_angle(-5*PI/2) == -PI/2);
    }    

    TEST_CASE("Test << operator for points", "[operator<<]"){
        Point2D point;
        point = {0, 1};

        std::ostringstream oss;
        oss << point;
        REQUIRE(oss.str() == "[0 1]");
    }

    TEST_CASE("Testing >> operator for points", "[operator>>]"){
        Point2D point;
        SECTION("test whether >> can handle and array input: '[0 1]'"){
            std::istringstream iss("[0 1]");
            iss >> point;   
            REQUIRE(point.x == 0);
            REQUIRE(point.y == 1);
        }

        SECTION("test wheter >> can handle an input of two doubles: '0 1'"){
            std::istringstream iss("0 1");
            iss >> point;   
            REQUIRE(point.x == 0);
            REQUIRE(point.y == 1);
        }
        
    }

    TEST_CASE("Testing - operator for points", "[operator-]"){
        Point2D head;
        Point2D tail;

        head = {10, 10};
        tail = {5, 5};

        Vector2D vec = head-tail;

        REQUIRE(vec.x == 5);
        REQUIRE(vec.y == 5);
    }

    TEST_CASE("Testing + operator for points", "[operator+]"){
        Point2D tail;
        Vector2D disp;
        
        tail = {0, 0};
        disp = {5, 5};

        Point2D head = tail + disp;

        REQUIRE(head.x == 5);
        REQUIRE(head.y == 5);
    }
    
    TEST_CASE("Testing << operator for vectors", "[operator<< vector]"){
        Vector2D vec;
        vec = {0, 1};

        std::ostringstream oss;
        oss << vec;
        REQUIRE(oss.str() == "[0 1]");
    }

    TEST_CASE("Testing >> operator for vectors", "[operator>> vector]"){
        Vector2D vec;
        SECTION("test whether >> can handle an input of two doubles: '[0 1]'"){
            std::istringstream iss("[0 1]");
            iss >> vec;
            REQUIRE(vec.x == 0);
            REQUIRE(vec.y == 1);
        }
        SECTION("test whether >> can handle an input of two doubles: '0 1'"){
            std::istringstream iss("0 1");
            iss >> vec;
            REQUIRE(vec.x == 0);
            REQUIRE(vec.y == 1);
        }
        
    }

    TEST_CASE("Test normalize_vector", "[normalize_vector]")
    {
        Vector2D v = {1.0, 1.0};
        Vector2D v_norm = normalize_vector(v);
        REQUIRE_THAT(v_norm.x, Catch::Matchers::WithinAbs(0.707107, 0.0001));
        REQUIRE_THAT(v_norm.y, Catch::Matchers::WithinAbs(0.707107, 0.0001));
    }

    TEST_CASE("Test * operator with a Point2D object and a double", "[operator*_point]")
    {
        Point2D p = {1.0, 1.0};
        p = p * 2.0;
        REQUIRE_THAT(p.x, Catch::Matchers::WithinAbs(2.0, 0.0001));
        REQUIRE_THAT(p.y, Catch::Matchers::WithinAbs(2.0, 0.0001));
    }

    TEST_CASE("Test * operator with a Vector2D object and a double", "[operator*_vector]")
    {
        Vector2D v = {1.0, 1.0};
        v = v * 2.0;
        REQUIRE_THAT(v.x, Catch::Matchers::WithinAbs(2.0, 0.0001));
        REQUIRE_THAT(v.y, Catch::Matchers::WithinAbs(2.0, 0.0001));
    }
}


