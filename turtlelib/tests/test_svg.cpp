#include"turtlelib/svg.hpp"
#include"turtlelib/se2d.hpp"
#include"turtlelib/geometry2d.hpp"
#include<catch2/catch_test_macros.hpp>
#include<catch2/matchers/catch_matchers_floating_point.hpp>
#include<string>

namespace turtlelib{
    TEST_CASE("Test that default constructor works", "[default_constructor]")
    {
        Svg svg;
        svg.close();

        bool same = true;

        std::ifstream test_file("default.svg");
        std::ifstream test_file_compare("default_comparison.svg");

        if (!test_file.is_open() || !test_file_compare.is_open()){
            same = false;
        }
        
        std::string test_line;
        std::string test_line_compare;

        while(std::getline(test_file, test_line) && std::getline(test_file_compare, test_line_compare)){
            if (test_line != test_line_compare){
                same = false;
                break;
            }
        }

        REQUIRE(same);
    }

    TEST_CASE("Test that the constructor that accepts a filenae works", "[filename_constructor]")
    {
        Svg svg("filename_constructor.svg");
        svg.close();

        bool same = true;

        std::ifstream test_file("filename_constructor.svg");
        std::ifstream test_file_compare("filename_constructor_comparison.svg");

        if (!test_file.is_open() || !test_file_compare.is_open()){
            same = false;
        }
        
        std::string test_line;
        std::string test_line_compare;

        while(std::getline(test_file, test_line) && std::getline(test_file_compare, test_line_compare)){
            if (test_line != test_line_compare){
                same = false;
                break;
            }
        }

        REQUIRE(same);
    }

    TEST_CASE("Test function left_to_right for Vector2D objects", "[left_to_right_vector]")
    {
        Svg svg;
        Vector2D v = {1,1};
        svg.left_to_right(v);
        REQUIRE(v.x == -1);
    }

    TEST_CASE("Test function left_to_right for Point2D objects", "[left_to_right_point]")
    {
        Svg svg;
        Point2D p = {1,1};
        svg.left_to_right(p);
        REQUIRE(p.x == -1);
    }

    TEST_CASE("Test that svg class can draw a point", "[point]")
    {
        Svg svg("point_test.svg");
        Point2D p = {0,1};
        svg.draw_point(p);
        svg.close();

        bool same = true;

        std::ifstream test_file("point_test.svg");
        std::ifstream test_file_compare("point_test_comparison.svg");

        if (!test_file.is_open() || !test_file_compare.is_open()){
            same = false;
        }
        
        std::string test_line;
        std::string test_line_compare;

        while(std::getline(test_file, test_line) && std::getline(test_file_compare, test_line_compare)){
            if (test_line != test_line_compare){
                same = false;
                break;
            }
        }

        REQUIRE(same);
    }

    TEST_CASE("Test that svg class can draw a point of a certain color", "[point_color]")
    {
        Svg svg("point_color_test.svg");
        Point2D p = {0,1};
        svg.draw_point(p, "purple");
        svg.close();

        bool same = true;

        std::ifstream test_file("point_color_test.svg");
        std::ifstream test_file_compare("point_color_test_comparison.svg");

        if (!test_file.is_open() || !test_file_compare.is_open()){
            same = false;
        }
        
        std::string test_line;
        std::string test_line_compare;

        while(std::getline(test_file, test_line) && std::getline(test_file_compare, test_line_compare)){
            if (test_line != test_line_compare){
                same = false;
                break;
            }
        }

        REQUIRE(same);
    }


    TEST_CASE("Test that svg class can draw a vector without specifying a starting point", "[vector_no_start]")
    {
        Svg svg("vector_no_start_test.svg");
        Vector2D v = {0, 1};
        svg.draw_vector(v);
        svg.close();

        bool same = true;

        std::ifstream test_file("vector_no_start_test.svg");
        std::ifstream test_file_compare("vector_no_start_test_comparison.svg");

        if (!test_file.is_open() || !test_file_compare.is_open()){
            same = false;
        }
        
        std::string test_line;
        std::string test_line_compare;

        while(std::getline(test_file, test_line) && std::getline(test_file_compare, test_line_compare)){
            if (test_line != test_line_compare){
                same = false;
                break;
            }
        }

        REQUIRE(same);
    }

    TEST_CASE("Test that svg class can draw a vector without specifying a starting point", "[vector_color]")
    {
        Svg svg("vector_no_start_test_color.svg");
        Vector2D v = {0, 1};
        svg.draw_vector(v, "purple");
        svg.close();

        bool same = true;

        std::ifstream test_file("vector_no_start_test_color.svg");
        std::ifstream test_file_compare("vector_no_start_test_color_comparison.svg");

        if (!test_file.is_open() || !test_file_compare.is_open()){
            same = false;
        }
        
        std::string test_line;
        std::string test_line_compare;

        while(std::getline(test_file, test_line) && std::getline(test_file_compare, test_line_compare)){
            if (test_line != test_line_compare){
                same = false;
                break;
            }
        }

        REQUIRE(same);
    }

    TEST_CASE("Test that svg class can draw a vector when specifying a starting point", "[vector_start]")
    {
        Svg svg("vector_start_test.svg");
        Vector2D v = {0, 1};
        Point2D tail = {2,2};
        svg.draw_vector(v, tail);
        svg.close();

        bool same = true;

        std::ifstream test_file("vector_start_test.svg");
        std::ifstream test_file_compare("vector_start_test_comparison.svg");

        if (!test_file.is_open() || !test_file_compare.is_open()){
            same = false;
        }
        
        std::string test_line;
        std::string test_line_compare;

        while(std::getline(test_file, test_line) && std::getline(test_file_compare, test_line_compare)){
            if (test_line != test_line_compare){
                same = false;
                break;
            }
        }

        REQUIRE(same);
    }

    TEST_CASE("Test that svg class can draw a vector when specifying a starting point", "[vector_start_color]")
    {
        Svg svg("vector_start_color_test.svg");
        Vector2D v = {0, 1};
        Point2D tail = {2,2};
        svg.draw_vector(v, tail, "purple");
        svg.close();

        bool same = true;

        std::ifstream test_file("vector_start_color_test.svg");
        std::ifstream test_file_compare("vector_start_color_test_comparison.svg");

        if (!test_file.is_open() || !test_file_compare.is_open()){
            same = false;
        }
        
        std::string test_line;
        std::string test_line_compare;

        while(std::getline(test_file, test_line) && std::getline(test_file_compare, test_line_compare)){
            if (test_line != test_line_compare){
                same = false;
                break;
            }
        }

        REQUIRE(same);
    }

    TEST_CASE("Test that svg class can draw a coordinate frame without specifying a start point", "[coordinate_frame_default]")
    {
        Svg svg("coordinate_frame_test_default.svg");

        svg.draw_coordinate_frame();
        svg.close();

        bool same = true;

        std::ifstream test_file("coordinate_frame_test_default.svg");
        std::ifstream test_file_compare("coordinate_frame_test_default_comparison.svg");

        if (!test_file.is_open() || !test_file_compare.is_open()){
            same = false;
        }
        
        std::string test_line;
        std::string test_line_compare;

        while(std::getline(test_file, test_line) && std::getline(test_file_compare, test_line_compare)){
            if (test_line != test_line_compare){
                same = false;
                break;
            }
        }
        REQUIRE(same);
    }

    TEST_CASE("Test that svg class can draw a coordinate frame while specifying a start point", "[coordinate_frame_point]")
    {
        Svg svg("coordinate_frame_test_point.svg");

        Transform2D t;
        svg.draw_coordinate_frame(t);
        svg.close();

        bool same = true;

        std::ifstream test_file("coordinate_frame_test_point.svg");
        std::ifstream test_file_compare("coordinate_frame_test_point_comparison.svg");

        if (!test_file.is_open() || !test_file_compare.is_open()){
            same = false;
        }
        
        std::string test_line;
        std::string test_line_compare;

        while(std::getline(test_file, test_line) && std::getline(test_file_compare, test_line_compare)){
            if (test_line != test_line_compare){
                same = false;
                break;
            }
        }

        REQUIRE(same);
    }

    TEST_CASE("Test that svg class can draw a coordinate frame while specifying a start point and specifying name", "[coordinate_frame_point_text]")
    {
        Svg svg("coordinate_frame_test_point_text.svg");

        Transform2D t;
        svg.draw_coordinate_frame(t, "b");
        svg.close();

        bool same = true;

        std::ifstream test_file("coordinate_frame_test_point_text.svg");
        std::ifstream test_file_compare("coordinate_frame_test_point_text_comparison.svg");

        if (!test_file.is_open() || !test_file_compare.is_open()){
            same = false;
        }
        
        std::string test_line;
        std::string test_line_compare;

        while(std::getline(test_file, test_line) && std::getline(test_file_compare, test_line_compare)){
            if (test_line != test_line_compare){
                same = false;
                break;
            }
        }

        REQUIRE(same);
    }
}