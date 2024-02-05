#include"turtlelib/geometry2d.hpp"
#include"turtlelib/se2d.hpp"
#include"turtlelib/svg.hpp"
#include<fstream>
#include<string>
#include<iostream>

namespace turtlelib
{
    Svg::Svg() : fixed_frame(page_center, fixed_frame_rad)
    {
        left_to_right(x_axis);
        left_to_right(y_axis);
        x_axis = fixed_frame(x_axis * ppi);
        y_axis = fixed_frame(y_axis * ppi);

        svgFile.open("default.svg", std::ios_base::trunc); // std::ios_base::out implicitly included
        svgFile << "<svg width=\"" << page_width << "in\" height=\"" << page_height << "in\" viewBox=\"0 0 " << page_width*ppi << " " << page_height*ppi << "\" xmlns=\"http://www.w3.org/2000/svg\">" << std::endl;
        svgFile << "<defs>" << std::endl;
        svgFile << "  <marker" << std::endl;
        svgFile << "    style=\"overflow:visible\"" << std::endl;
        svgFile << "    id=\"Arrow1Sstart\"" << std::endl;
        svgFile << "    refX=\"0.0\"" << std::endl;
        svgFile << "    refY=\"0.0\"" << std::endl;
        svgFile << "    orient=\"auto\">" << std::endl;
        svgFile << "      <path" << std::endl;
        svgFile << "        transform=\"scale(0.2) translate(6,0)\"" << std::endl;
        svgFile << "        style=\"fill-rule:evenodd;fill:context-stroke;stroke:context-stroke;stroke-width:1.0pt\"" << std::endl;
        svgFile << "        d=\"M 0.0,0.0 L 5.0,-5.0 L -12.5,0.0 L 5.0,5.0 L 0.0,0.0 z \"" << std::endl;
        svgFile << "        />" << std::endl;
        svgFile << "    </marker>" << std::endl;
        svgFile << "</defs>";
    }

    Svg::Svg(std::string fileName) : fixed_frame(page_center, fixed_frame_rad)
    {
        left_to_right(x_axis);
        left_to_right(y_axis);
        x_axis = fixed_frame(x_axis * ppi);
        y_axis = fixed_frame(y_axis * ppi);

        svgFile.open(fileName, std::ios_base::trunc); // std::ios_base::out implicitly included
        svgFile << "<svg width=\"" << page_width << "in\" height=\"" << page_height << "in\" viewBox=\"0 0 " << page_width*ppi << " " << page_height*ppi << "\" xmlns=\"http://www.w3.org/2000/svg\">" << std::endl;
        svgFile << "<defs>" << std::endl;
        svgFile << "  <marker" << std::endl;
        svgFile << "    style=\"overflow:visible\"" << std::endl;
        svgFile << "    id=\"Arrow1Sstart\"" << std::endl;
        svgFile << "    refX=\"0.0\"" << std::endl;
        svgFile << "    refY=\"0.0\"" << std::endl;
        svgFile << "    orient=\"auto\">" << std::endl;
        svgFile << "      <path" << std::endl;
        svgFile << "        transform=\"scale(0.2) translate(6,0)\"" << std::endl;
        svgFile << "        style=\"fill-rule:evenodd;fill:context-stroke;stroke:context-stroke;stroke-width:1.0pt\"" << std::endl;
        svgFile << "        d=\"M 0.0,0.0 L 5.0,-5.0 L -12.5,0.0 L 5.0,5.0 L 0.0,0.0 z \"" << std::endl;
        svgFile << "        />" << std::endl;
        svgFile << "    </marker>" << std::endl;
        svgFile << "</defs>";
    }

    void Svg::draw_point(Point2D p)
    {
        left_to_right(p);
        p = fixed_frame(p * ppi);
   
        svgFile << "<circle cx=\"" << p.x << "\" cy=\"" << p.y << "\" r=\"3\" stroke=\"purple\" fill=\"purple\" stroke-width=\"1\"/>" << std::endl;
    }

    void Svg::draw_point(Point2D p, std::string color)
    {
        left_to_right(p);
        p = fixed_frame(p * ppi);
   
        svgFile << "<circle cx=\"" << p.x << "\" cy=\"" << p.y << "\" r=\"3\" stroke=\"" << color << "\" fill=\"" << color << "\" stroke-width=\"1\"/>" << std::endl;
    }

    void Svg::draw_vector(Vector2D v)
    {   
        left_to_right(v);

        Point2D tail = {0,0};
        tail =  fixed_frame(tail * ppi);

        v = fixed_frame(v * ppi);

        Point2D head = tail + v;

        svgFile << "<line x1=\"" << head.x << "\" x2=\"" << tail.x << "\" y1=\"" << head.y << "\" y2=\"" << tail.y << "\" stroke=\"purple\" stroke-width=\"5\" marker-start=\"url(#Arrow1Sstart)\" />" << std::endl;
    }

    void Svg::draw_vector(Vector2D v, std::string color)
    {
        left_to_right(v);

        Point2D tail = {0,0};
        tail =  fixed_frame(tail * ppi);

        v = fixed_frame(v * ppi);

        Point2D head = tail + v;

        svgFile << "<line x1=\"" << head.x << "\" x2=\"" << tail.x << "\" y1=\"" << head.y << "\" y2=\"" << tail.y << "\" stroke=\"" << color << "\" stroke-width=\"5\" marker-start=\"url(#Arrow1Sstart)\" />" << std::endl;
    }

    void Svg::draw_vector(Vector2D v, Point2D tail)
    {
        left_to_right(v);
        left_to_right(tail);

        tail = fixed_frame(tail * ppi);
        v = fixed_frame(v * ppi);
        Point2D head = tail + v;

        

        svgFile << "<line x1=\"" << head.x << "\" x2=\"" << tail.x << "\" y1=\"" << head.y << "\" y2=\"" << tail.y << "\" stroke=\"purple\" stroke-width=\"5\" marker-start=\"url(#Arrow1Sstart)\" />" << std::endl;
    }

    void Svg::draw_vector(Vector2D v, Point2D tail, std::string color)
    {
        left_to_right(v);
        left_to_right(tail);

        tail = fixed_frame(tail * ppi);
        v = fixed_frame(v * ppi);
        Point2D head = tail + v;

        svgFile << "<line x1=\"" << head.x << "\" x2=\"" << tail.x << "\" y1=\"" << head.y << "\" y2=\"" << tail.y << "\" stroke=\"" << color << "\" stroke-width=\"5\" marker-start=\"url(#Arrow1Sstart)\" />" << std::endl;
    }

    void Svg::draw_coordinate_frame()
    {
        Point2D p = {0,0};
        p = fixed_frame(p);

        Point2D headx = p + x_axis;
        Point2D heady = p + y_axis;
        
        svgFile << "<g>" << std::endl;
        svgFile << "<line x1=\"" << headx.x << "\" x2=\"" << p.x << "\" y1=\"" << headx.y << "\" y2=\"" << p.y << "\" stroke=\"red\" stroke-width=\"5\" marker-start=\"url(#Arrow1Sstart)\" />" << std::endl;
        svgFile << "<line x1=\"" << heady.x << "\" x2=\"" << p.x << "\" y1=\"" << heady.y << "\" y2=\"" << p.y << "\" stroke=\"green\" stroke-width=\"5\" marker-start=\"url(#Arrow1Sstart)\" />" << std::endl;
        svgFile << "<text x=\"" << p.x << "\" y=\"" << p.y + 0.25 << "\">{a}</text>" << std::endl;
        svgFile << "</g>" << std::endl;
    }

    void Svg::draw_coordinate_frame(Transform2D t)
    {
        Point2D p = {t.translation().x, t.translation().y};
        left_to_right(p);

        p = fixed_frame(p * ppi);
        Point2D headx = p + x_axis;
        Point2D heady = p + y_axis;
        
        svgFile << "<g>" << std::endl;
        svgFile << "<line x1=\"" << headx.x << "\" x2=\"" << p.x << "\" y1=\"" << headx.y << "\" y2=\"" << p.y << "\" stroke=\"red\" stroke-width=\"5\" marker-start=\"url(#Arrow1Sstart)\" />" << std::endl;
        svgFile << "<line x1=\"" << heady.x << "\" x2=\"" << p.x << "\" y1=\"" << heady.y << "\" y2=\"" << p.y << "\" stroke=\"green\" stroke-width=\"5\" marker-start=\"url(#Arrow1Sstart)\" />" << std::endl;
        svgFile << "<text x=\"" << p.x << "\" y=\"" << p.y + 0.25 << "\">{a}</text>" << std::endl;
        svgFile << "</g>" << std::endl;
    }

    void Svg::draw_coordinate_frame(Transform2D t, std::string text)
    {
        // Here I am translating first, THEN rotating
        
        Point2D p = {t.translation().x, t.translation().y};
        p = fixed_frame(p * ppi);

        double radians = -t.rotation(); // const auto
        Transform2D t_new(radians);

        Point2D headx = p + t_new(x_axis);
        Point2D heady = p + t_new(y_axis);

        svgFile << "<g>" << std::endl;
        svgFile << "<line x1=\"" << headx.x << "\" x2=\"" << p.x << "\" y1=\"" << headx.y << "\" y2=\"" << p.y << "\" stroke=\"red\" stroke-width=\"5\" marker-start=\"url(#Arrow1Sstart)\" />" << std::endl;
        svgFile << "<line x1=\"" << heady.x << "\" x2=\"" << p.x << "\" y1=\"" << heady.y << "\" y2=\"" << p.y << "\" stroke=\"green\" stroke-width=\"5\" marker-start=\"url(#Arrow1Sstart)\" />" << std::endl;
        svgFile << "<text x=\"" << p.x << "\" y=\"" << p.y + 0.25 << "\">{" << text << "}</text>" << std::endl;
        svgFile << "</g>" << std::endl;
    } 

    void Svg::close(){
        svgFile << "</svg>";
        svgFile.close(); // I know that this is called implicitly when the object
                         // is deconstructed, however I need the file to close earlier than this
                         // for testing.
    }
}
