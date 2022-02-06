#include "turtlelib/rigid2d.hpp"
#include <iostream>
#include <math.h> 

namespace turtlelib
{

// -------Normalize theta ---------
double normalize_angle(double angle)
{
    double x = fmod(angle-PI,2*PI);
    if (x > 0){
        x -= 2*PI;
    }
    return x + PI;
}

// ----- Vector2D struct ----------

Vector2D Vector2D::normalize() const
{
    double n = sqrt(x*x + y*y);
    Vector2D newv;
    newv.x = x/n;
    newv.y = y/n;
    return newv;
}

Vector2D & Vector2D::operator+=(const Vector2D & v_rhs)
{
    x += v_rhs.x;
    y += v_rhs.y;
    
    return *this;
}

Vector2D & Vector2D::operator-=(const Vector2D & v_rhs)
{
    x -= v_rhs.x;
    y -= v_rhs.y;
    
    return *this;
}

Vector2D & Vector2D::operator*=(const double scalar)
{
    x *= scalar;
    y *= scalar;
    
    return *this;
}

double Vector2D::dot(Vector2D &v1, Vector2D &v2) const
{
    double scalar = v1.x*v2.x + v1.y*v2.y;
    return scalar;
}

double Vector2D::magnitude(Vector2D &v) const
{
    double mag2 = v.x*v.x + v.y*v.y;
    double mag = sqrt(mag2);
    return mag;
}

double Vector2D::angle(Vector2D &v1, Vector2D &v2) const
{
    double theta = acos(dot(v1,v2)/(magnitude(v1)+magnitude(v2)));
    return theta;
}
// Vector2D related operators
Vector2D operator+(Vector2D lhs, const Vector2D & rhs)
{
    lhs += rhs;
    return lhs;
    
}

Vector2D operator-(Vector2D lhs, const Vector2D & rhs)
{
    lhs -= rhs;
    return lhs;
    
}

Vector2D operator*(Vector2D lhs, const double & scalar)
{
    lhs *= scalar;
    return lhs;
    
}

// ----- Transform2D class --------

Transform2D::Transform2D()
{
    vec.x = 0.0;
    vec.y = 0.0;
    theta = 0.0;

}

Transform2D::Transform2D(Vector2D trans)
{
    vec.x = trans.x;
    vec.y = trans.y;
    theta = 0.0;

}

Transform2D::Transform2D(double radians)
{
    vec.x = 0.0;
    vec.y = 0.0;
    theta = radians;

}

Transform2D::Transform2D(Vector2D trans, double radians)
{
    vec.x = trans.x;
    vec.y = trans.y;
    theta = radians;

}

Vector2D Transform2D::operator()(Vector2D v) const
{
    Vector2D newv;
    newv.x = cos(theta)*v.x - sin(theta)*v.y + vec.x;
    newv.y = sin(theta)*v.x + cos(theta)*v.y + vec.y;

    return newv;
}

Twist Transform2D::operator()(Twist twist) const
{
    Twist newtw;
    newtw.thetadot = twist.thetadot;
    newtw.xdot = vec.y*twist.thetadot + cos(theta)*twist.xdot - sin(theta)*twist.ydot;
    newtw.ydot = -vec.x*twist.thetadot + sin(theta)*twist.xdot + cos(theta)*twist.ydot;

    return newtw;
}


Transform2D Transform2D::inv() const
{
    Vector2D inv_vec;
    inv_vec.x = -vec.x*cos(theta) - vec.y*sin(theta);
    inv_vec.y = -vec.y*cos(theta) + vec.x*sin(theta);

    // copy constructor
    Transform2D inv_trans = Transform2D(inv_vec, -theta);
    return inv_trans;

}

Transform2D & Transform2D::operator*=(const Transform2D &rhs)
{
    Vector2D rhs_vec;
    rhs_vec.x = rhs.vec.x;
    rhs_vec.y = rhs.vec.y;
    double rhs_theta = rhs.theta;   

    vec.x = vec.x + cos(theta)*rhs_vec.x - sin(theta)*rhs_vec.y;
    vec.y = vec.y + sin(theta)*rhs_vec.x + cos(theta)*rhs_vec.y;
    theta = theta + rhs_theta;

    return *this;
}


Vector2D Transform2D::translation() const
{
    
    Vector2D translation;
    translation.x = vec.x;
    translation.y = vec.y;

    return translation;
    
}

double Transform2D::rotation() const
{
    double angle = theta;

    return angle;
}



// --------------------------------------------------------
// Integrate a twist to give the transformation in the body frame
Transform2D integrate_twist(Twist V)
{

    Vector2D v_s;

    if(V.thetadot==0.0)
    {
        v_s.x = V.xdot;
        v_s.y = V.ydot;
        Transform2D Tbb_hat(v_s);
        return Tbb_hat;
    }

    double theta_s = V.thetadot;
    v_s.x = V.ydot/theta_s;
    v_s.y = -V.xdot/theta_s;

    Transform2D Tbs(v_s), Tss_hat(theta_s), Tbb_hat; 
    Tbb_hat = Tbs*Tss_hat*Tbs;
    return Tbb_hat;

}

// Output a 2 dimensional vector as [xcomponent ycomponent]
std::ostream & operator<<(std::ostream & os, const Vector2D & v)
{
    os << "["<<v.x<<", "<< v.y<<"]\n";
    return os;
}

// Input a 2 dimensional vector
std::istream & operator>>(std::istream & is, Vector2D & v)
{
    // std::cout << "Enter vector [x y]:" << std::endl;
    char c = is.peek();
    if( c == '[' )
    {
        is.get();
    }

    is >> v.x >> v.y;
    // std::cout << "[" << v.x<<", "<< v.y<<"]\n";
    char c1 = is.peek();
    if( c1 == ']' )
    {
        is.get();
    }
    is.get();
    return is;

}

std::ostream & operator<<(std::ostream & os, const Transform2D & tf)
{
    os << "deg: " << rad2deg(tf.theta) << " x: " <<tf.vec.x<<" y: "<< tf.vec.y << std::endl;
    return os;
}


std::istream & operator>>(std::istream & is, Transform2D & tf)
{
    Vector2D vector;
    double theta;
    std::string s1, s2, s3;

    is >> s1 >> theta >> s2 >> vector.x >> s3 >> vector.y;

    is.get();

    theta = deg2rad(theta);
    tf = Transform2D(vector,theta);

    return is;
}

Transform2D operator*(Transform2D lhs, const Transform2D & rhs)
{
    Transform2D mult;
    lhs *= rhs;
    mult = lhs;
    return mult;
}

/// TWIST 
// Output a 2 dimensional vector as [xcomponent ycomponent]
std::ostream & operator<<(std::ostream & os, const Twist & twist)
{
    os << "["<<twist.thetadot<<", "<<twist.xdot<<", "<< twist.ydot<<"]\n";
    return os;
}

// Input a 3 dimensional twist
std::istream & operator>>(std::istream & is, Twist & twist)
{
    char c = is.peek();
    if(c=='[')
    {
        is.get();
    }

    is >> twist.thetadot >> twist.xdot >> twist.ydot;
    char c1 = is.peek();
    if( c1 == ']' )
    {
        is.get();
    }
    is.get();
    return is;

}


}