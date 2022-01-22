#include "../include/turtlelib/rigid2d.hpp"
#include <iostream>

int main()
{
    turtlelib::Transform2D T_ab, T_ba, T_bc, T_cb, T_ac, T_ca;
    turtlelib::Vector2D v_b, v_a, v_c, v_bhat;
    turtlelib::Twist V_a, V_b, V_c;

    std::cout << "Enter transform T_{a,b}:" << std::endl;
    std::cin >> T_ab;

    std::cout << "Enter transform T_{b,c}:" << std::endl;
    std::cin >> T_bc;


    std::cout <<"T_{a,b}: " << T_ab << std::endl;
    T_ba = T_ab.inv();
    std::cout <<"T_{b,a}: " << T_ba << std::endl;

    std::cout <<"T_{b,c}: " << T_bc << std::endl;
    T_cb = T_bc.inv();
    std::cout <<"T_{c,b}: " << T_cb << std::endl;

    T_ac = T_ab*T_bc;
    T_ca = T_ac.inv();

    std::cout <<"T_{a,c}: " << T_ac << std::endl;
    std::cout <<"T_{c,a}: " << T_ca << std::endl;

    std::cout << "Enter vector v_b:" << std::endl;
    std::cin >> v_b;

    v_c = T_cb(v_b);
    v_a = T_ab(v_b);
    v_bhat = v_b.normalize();

    std::cout << "v_bhat:" << v_bhat << std::endl;
    std::cout << "v_a:" << v_a << std::endl;
    std::cout << "v_b:" << v_b << std::endl;
    std::cout << "v_c:" << v_c << std::endl;

    std::cout << "Enter Twist V_b:" << std::endl;
    std::cin >> V_b;

    V_c = T_cb(V_b);
    V_a = T_ab(V_b);

    std::cout << "V_a:" << V_a << std::endl;
    std::cout << "V_b:" << V_b << std::endl;
    std::cout << "V_c:" << V_c << std::endl;

}