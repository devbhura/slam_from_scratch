#include "rigid2d.hpp"
#include <iostream>

// ----- Transform2D class --------

turtlelib::Transform2D::Transform2D()
{
    vec.x = 0.0;
    vec.y = 0.0;
    theta = 0.0;

}

turtlelib::Transform2D::Transform2D(Vector2D trans)
{
    vec.x = trans.x;
    vec.y = trans.y;
    theta = 0.0;

}

turtlelib::Transform2D::Transform2D(double radians)
{
    vec.x = 0.0;
    vec.y = 0.0;
    theta = radians;

}

turtlelib::Transform2D::Transform2D(Vector2D trans, double radians)
{
    vec.x = trans.x;
    vec.y = trans.y;
    theta = radians;

}

turtlelib::Vector2D turtlelib::Transform2D::operator()(turtlelib::Vector2D v) const
{
    turtlelib::Vector2D newv;
    newv.x = cos(theta)*v.x - sin(theta)*v.y + vec.x;
    newv.y = sin(theta)*v.x + cos(theta)*v.y + vec.y;

    return newv;
}

turtlelib::Vector2D turtlelib::Transform2D::operator()(turtlelib::Vector2D v) const
{
    turtlelib::Vector2D newv;
    newv.x = cos(theta)*v.x - sin(theta)*v.y + vec.x;
    newv.y = sin(theta)*v.x + cos(theta)*v.y + vec.y;

    return newv;
}

turtlelib::Transform2D turtlelib::Transform2D::inv() const
{
    turtlelib::Vector2D inv_vec;
    inv_vec.x = -vec.x*cos(theta) - vec.y*sin(theta);
    inv_vec.y = -vec.y*cos(theta) + vec.x*sin(theta);

    // copy constructor
    turtlelib::Transform2D inv_trans = turtlelib::Transform2D(inv_vec, -theta);
    return inv_trans;

}

turtlelib::Transform2D & turtlelib::Transform2D::operator*=(const turtlelib::Transform2D &rhs)
{
    
}



int main(){
    std::cout<< "done\n";
  return 0;  
}