#define CATCH_CONFIG_MAIN
#include "catch.hpp"
#include "../include/turtlelib/rigid2d.hpp"

#include <cmath>

TEST_CASE( "Test for rotation", "rotation" ) {

    turtlelib::Transform2D Tab;

    double theta_check = Tab.rotation();

    REQUIRE(turtlelib::almost_equal(theta_check,0.0,0.01));
}

TEST_CASE( "Test for translation", "translation" ) {

    turtlelib::Transform2D Tab;

    turtlelib::Vector2D v0 = Tab.translation();

    REQUIRE(turtlelib::almost_equal(v0.x,0.0,0.01));
    REQUIRE(turtlelib::almost_equal(v0.y,0.0,0.01));
}


TEST_CASE( "Test for rotation transform", "rotation transform" ) {

    double theta = turtlelib::deg2rad(90);
    turtlelib::Vector2D v, v0;
    turtlelib::Transform2D Tab(theta);

    double theta_check;
    v0 = Tab.translation();
    theta_check = Tab.rotation();

    REQUIRE(turtlelib::almost_equal(theta,theta_check,0.01));
    REQUIRE(turtlelib::almost_equal(v.x,v0.x,0.01));
    REQUIRE(turtlelib::almost_equal(v.y,v0.y,0.01));
}

TEST_CASE( "Test for translation transform", "translation transform" ) {

    turtlelib::Vector2D v, v0;
    v.x = 1.0;
    v.y = 1.0;
    turtlelib::Transform2D Tab(v);

    v0 = Tab.translation();
    double theta_check = Tab.rotation();

    REQUIRE(turtlelib::almost_equal(theta_check,0.0,0.01));
    REQUIRE(turtlelib::almost_equal(v.x,v0.x,0.01));
    REQUIRE(turtlelib::almost_equal(v.y,v0.y,0.01));
}

TEST_CASE( "Test for Identity", "Identity" ) {

    turtlelib::Vector2D v0;
    turtlelib::Transform2D Tab;

    v0 = Tab.translation();
    double theta_check = Tab.rotation();

    REQUIRE(turtlelib::almost_equal(theta_check,0.0,0.01));
    REQUIRE(turtlelib::almost_equal(v0.x,0.0,0.01));
    REQUIRE(turtlelib::almost_equal(v0.y,0.0,0.01));
}

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


TEST_CASE( "Test for vector transformation", "Vector Transform" ) {

    turtlelib::Vector2D v, v0, vt;
    double theta = turtlelib::deg2rad(90);
    v.x = 1.0;
    v.y = 1.0;
    turtlelib::Transform2D Tab(v,theta);
    vt.x = 0.0;
    vt.y = 0.0;
    v0 = Tab(vt);

    REQUIRE(turtlelib::almost_equal(v0.x,v.x,0.01));
    REQUIRE(turtlelib::almost_equal(v0.y,v.y,0.01));
}

TEST_CASE( "Test for Twist transformation", "Twist Transform" ) {

    turtlelib::Twist V0, Vt;
    turtlelib::Vector2D v;
    double theta = turtlelib::deg2rad(90);
    v.x = 1.0;
    v.y = 1.0;
    turtlelib::Transform2D Tab(v,theta);
    Vt.xdot = 0.0;
    Vt.ydot = 0.0;
    Vt.thetadot = 0.0;
    V0 = Tab(Vt);

    REQUIRE(turtlelib::almost_equal(V0.xdot,Vt.xdot,0.01));
    REQUIRE(turtlelib::almost_equal(V0.ydot,Vt.ydot,0.01));
    REQUIRE(turtlelib::almost_equal(V0.thetadot,Vt.thetadot,0.01));
}


TEST_CASE( "Test for Multiplication", "Multiplication" ) {

    turtlelib::Vector2D v, v0, vt;
    double theta0;
    double theta = turtlelib::deg2rad(90);
    v.x = 1.0;
    v.y = 1.0;
    turtlelib::Transform2D Tab(v,theta);
    

    REQUIRE(turtlelib::almost_equal(V0.xdot,Vt.xdot,0.01));
    REQUIRE(turtlelib::almost_equal(V0.ydot,Vt.ydot,0.01));
    REQUIRE(turtlelib::almost_equal(V0.thetadot,Vt.thetadot,0.01));
}