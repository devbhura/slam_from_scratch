#define CATCH_CONFIG_MAIN
#include "catch.hpp"
#include "../include/turtlelib/rigid2d.hpp"

#include <cmath>


TEST_CASE( "Test for inv", "[inverse]" ) {

    double theta = turtlelib::deg2rad(90);
    turtlelib::Vector2D v, v0;
    v.x = 0;
    v.y = 0;
    turtlelib::Transform2D Tab(v,theta), Tba;

    v0 = Tab(v);
    

    REQUIRE(turtlelib::almost_equal(v.x,v0.x,0.1));
    REQUIRE(turtlelib::almost_equal(v.y,v0.y,0.1));
}

