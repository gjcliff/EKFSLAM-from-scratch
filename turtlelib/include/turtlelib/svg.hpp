#ifndef TURTLELIB_SVG_INCLUDE_GUARD_HPP
#define TURTLELIB_SVG_INCLUDE_GUARD_HPP
/// \file
/// \brief Draw points, vectors, and coordinate frames

#include<iosfwd> // contains forward definitions for iostream objects
#include"turtlelib/geometry2d.hpp"
#include"turtlelib/se2d.hpp"
#include<fstream>
#include<string>

namespace turtlelib
{

    /// \brief draw points and vectors in an svg file
    class Svg
    {
    public:
        /// \brief create an svg file with the default name, "default.svg",
        /// delete any previous contents, and open it for writing
        Svg();

        /// \brief create a svg file with a specific file name, delete its
        ///contents, and open it for writing
        /// \param fileName - the file name. MUST append .svg
        Svg(std::string fileName); // const std::string &

        /// \brief flip from a left-handed coordinate system to a
        /// right-handed coordinate system
        /// \param v - the vector to be flipped
        constexpr void left_to_right(Vector2D & v){
            v.x *= -1;
        }
        /// @brief flip the point from a left-handed coordinate system to a
        /// right-handed coordinate system
        /// @param p 
        constexpr void left_to_right(Point2D & p){
            p.x *= -1;
        }

        /// \brief draw a point in the svg file
        /// \param p - the coordinates of the point that will be drawn
        void draw_point(Point2D p);

        /// @brief draw a point in the svg file of a certain color
        /// @param p  - the coordaites of the point that will be drawn
        /// @param color - the color of the point that will be draw
        void draw_point(Point2D p, std::string color); // std string by ref or std::string_view

        /// \brief draw a vector in the svg file from the origin
        /// \param v - the vector that will be drawn
        void draw_vector(Vector2D v);

        /// @brief draw a vector in the svg file from the origin in a specific color
        /// @param v - the vector to be drawn
        /// @param color - the color the vector will be
        void draw_vector(Vector2D v, std::string color); // std string by ref or std::string_view

        /// \brief draw a vector in the svg file starting from a point
        /// \param v - the vector that will be drawn
        /// \param tail - the point the vector will start from
        void draw_vector(Vector2D v, Point2D tail);

        /// \brief draw a vector in the svg file starting from a point
        /// \param v - the vector that will be drawn
        /// \param tail - the point the vector will start from
        /// \param color - the color the vector will be
        void draw_vector(Vector2D v, Point2D tail, std::string color);

        /// \brief draw a default coordinate frame in the svg file at the
        /// center of the page. The name will be "{a}"
        void draw_coordinate_frame();

        /// \brief draw a default coordinate frame in the svg file at the
        /// center of the page with axes length "v". The name will be "{a}"
        /// \param t - the origin of the coordinate axis
        void draw_coordinate_frame(Transform2D t);

        /// \brief draw a default coordinate frame in the svg file at the
        /// center of the page with axes length "v". The name will be "{a}"
        /// \param t - the origin of the coordinate axis
        /// \param text - the name of the coordinate axis, this text will be surrounded by
        /// curly brackets
        void draw_coordinate_frame(Transform2D t, std::string text);

        /// @brief write the closing </svg> tag to file, and close the file
        void close(); // is this svg object valid after calling .close?


    private:
        std::ofstream svgFile;

        int ppi = 96; // pixels per inch
        double page_width = 8.5;
        double page_height = 11.0;
        Vector2D page_center = {page_width/2*ppi, page_height/2*ppi}; // /2.0 no by 2

        double fixed_frame_rad = deg2rad(180);
        Transform2D fixed_frame;

        Vector2D x_axis = {0.5,0};
        Vector2D y_axis = {0,0.5};
    };
}
#endif
