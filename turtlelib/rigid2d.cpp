#include "rigid2d.hpp"
#include <iostream>
#include <math.h> 

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
    turtlelib::Vector2D rhs_vec;
    rhs_vec.x = rhs.vec.x;
    rhs_vec.y = rhs.vec.y;
    double rhs_theta = rhs.theta;

    turtlelib::Vector2D lhs_vec;
    lhs_vec.x = vec.x;
    lhs_vec.y = vec.y;
    double lhs_theta = theta;

    vec.x = lhs_vec.x + cos(rhs_theta)*rhs_vec.x - sin(rhs_theta)*rhs_vec.y;
    vec.y = lhs_vec.y + sin(rhs_theta)*rhs_vec.x + cos(rhs_theta)*rhs_vec.y;
    theta = lhs_theta + rhs_theta;

    return *this;
}


turtlelib::Vector2D turtlelib::Transform2D::translation() const
{
    
    turtlelib::Vector2D translation;
    translation.x = vec.x;
    translation.y = vec.y;

    return translation;
    
}

double turtlelib::Transform2D::rotation() const
{
    double angle = theta;

    return angle;
}

// Output a 2 dimensional vector as [xcomponent ycomponent]
std::ostream & turtlelib::operator<<(std::ostream & os, const turtlelib::Vector2D & v)
{
    os << "["<<v.x<<", "<< v.y<<"]\n";
    return os;
}

// Input a 2 dimensional vector
std::istream & turtlelib::operator>>(std::istream & is, turtlelib::Vector2D & v)
{
    // std::cout << "Enter vector [x y]:" << std::endl;
    char c = is.peek();
    if(c=='[')
    {
        is.get();
    }

    is >> v.x >> v.y;
    // std::cout << "[" << v.x<<", "<< v.y<<"]\n";

    return is;

}

std::ostream & turtlelib::operator<<(std::ostream & os, const turtlelib::Transform2D & tf)
{
    os << "deg: " << rad2deg(tf.theta) << " x: " <<tf.vec.x<<" y: "<< tf.vec.y << std::endl;
    return os;
}


std::istream & turtlelib::operator>>(std::istream & is, turtlelib::Transform2D & tf)
{
    turtlelib::Vector2D vector;
    double theta;

    std::cout << "deg: ";
    is >> theta;
    std::cout << " x: ";
    is >> vector.x;
    std::cout << " y: ";
    is >> vector.y;

    // is >> "deg:" >> theta >> "x:" >> vector.x >> "y:">> vector.y;

    theta = deg2rad(theta);
    tf = turtlelib::Transform2D(vector,theta);

    return is;
}

turtlelib::Transform2D turtlelib::operator*(turtlelib::Transform2D lhs, const turtlelib::Transform2D & rhs)
{
    turtlelib::Transform2D mult;
    lhs *= rhs;
    mult = lhs;
    return mult;
}

/// TWIST 
// Output a 2 dimensional vector as [xcomponent ycomponent]
std::ostream & turtlelib::operator<<(std::ostream & os, const turtlelib::Twist & twist)
{
    os << "["<<twist.thetadot<<", "<<twist.xdot<<", "<< twist.ydot<<"]\n";
    return os;
}

// Input a 2 dimensional vector
std::istream & turtlelib::operator>>(std::istream & is, turtlelib::Twist & twist)
{
    // std::cout << "Enter twist [thetadot xdot ydot]:" << std::endl;
    char c = is.peek();
    if(c=='[')
    {
        is.get();
    }

    is >> twist.thetadot >> twist.xdot >> twist.ydot;
    // std::cout << "[" << twist.thetadot<<", "<<twist.xdot<<", "<< twist.ydot<<"]\n";

    return is;

}


// int main(){
//     std::cout<< "done\n";
//   return 0;  
// }