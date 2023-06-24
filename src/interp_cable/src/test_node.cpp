#include <interp_cable/utilities.hpp>
#include <interp_cable/paraspline.hpp>
#include <interp_cable/matplotlibcpp.h>

namespace plt = matplotlibcpp;

int main()
{

    std::vector<double> X = Linspace(0, 1, 5); // must be increasing
    std::vector<double> Y = {0.0, 1.0, 2.0, 1.0, 0.0};
    // for (auto x : t){
    //     std::cout << x << std::endl;
    // }

    Cable::paraspline ps;
    ps.set_boundary(Cable::paraspline::bound_type::first_order, 0.0,
                    Cable::paraspline::bound_type::first_order, 0.0);

    // vector<double> x = {0, 1, 2, 1, 0};

    // ps.set_boundary(Cable::paraspline::bound_type::second_order, 0.0,
                    // Cable::paraspline::bound_type::second_order, 0.0);
    ps.set_points(X, Y, Cable::paraspline::cubic);
    int n = 1000;

    std::vector<double> plot_x;
    std::vector<double> plot_y;
    auto fine_t = Linspace(0, 1, n);
    for (int i = 0; i < n; i++)
    {
        std::cout << fine_t[i] << std::endl;
        plot_x.push_back(fine_t[i]);
        plot_y.push_back(ps(fine_t[i]));
        // ROS_INFO("So far so good");
    }
    // for (int i = 0; i < n; i++)
    // {
    //     std::cout << ps(fine_t[i]) << std::endl;
    //     // ROS_INFO("So far so good");
    // }



    plt::plot3(fine_t,plot_x,plot_y);
    plt::show();
    // for (double value = 0.0; value <= 1.0; value += 0.25)
    // {
    //     double y = ps(value), deriv = ps.deriv(1, value), deriv2 = ps.deriv(2, value);
    //     std::cout << "Function at: " << value
    //               << ", y: " << y
    //               << ", deriv: " << deriv
    //               << ", deriv2: " << deriv2
    //               << std::endl;
    // }

    return 0;
}