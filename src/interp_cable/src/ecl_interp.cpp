#include <ros/ros.h>
#include <ecl/geometry.hpp>
#include <array_dynamic_mem_check.hpp>
#include <iostream>

using ecl::CubicSpline;

void functionA(double x){
    std::cout<<x;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ecl_interp");
    ros::NodeHandle nh;
    // ros::Subscriber sub = nh.subscribe("xxx", 1, xxx);
    Array<double> x_set(5);
    Array<double> y_set(5);
    x_set = 0.0, 1.0, 2.0, 3.0, 4.0;
    y_set = 1.0, 2.0, 1.0, 3.0, 4.0;
    CubicSpline cubic;
    cubic = CubicSpline::Natural(x_set, y_set);
    cubic = CubicSpline::ContinuousDerivatives(x_set, y_set, ydot_0, ydot_f);
    cubic = CubicSpline::DerivativeHeuristic(x_set, y_set, ydot_0, ydot_f);
    std::cout<<cubic(3.2)<<std::endl;
    // ros::spin();
    return 0;
}