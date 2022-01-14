#include "rigid2d.hpp"
#include <iostream>

// ----- Transform2D class --------

turtlelib::Transform2D::Transform2D()
{
    vec.x = 0.0;
    vec.y = 0.0;
    angle = 0.0;

}

turtlelib::Transform2D::Transform2D(Vector2D trans)
{
    vec.x = trans.x;
    vec.y = trans.y;
    angle = 0.0;

}

turtlelib::Transform2D::Transform2D(double radians)
{
    vec.x = 0.0;
    vec.y = 0.0;
    angle = radians;

}

turtlelib::Transform2D::Transform2D(Vector2D trans, double radians)
{
    vec.x = trans.x;
    vec.y = trans.y;
    angle = radians;

}




int main(){
    std::cout<< "done\n";
  return 0;  
}