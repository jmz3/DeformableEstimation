#include <sorting/utilities.hpp>
#include <sorting/paraspline.hpp>

int main()
{

    auto t = linspace(0, 1, 5);

    // for (auto x : t){
    //     std::cout << x << std::endl;
    // }

    Cable::paraspline ps;
    ps.set_boundary(Cable::paraspline::bound_type::first_order, 0, Cable::paraspline::bound_type::first_order, 0);

    vector<double> x = {0, 2, 4, 2, 0};

    ps.set_points(t, x, Cable::paraspline::cubic);

    int n = 100;

    auto fine_t = linspace(0, 1, n);
    for (int i = 0; i < n; i++)
    {
        std::cout << fine_t[i] <<  std::endl;
        std::cout << ps(fine_t[i]) << std::endl;
        // ROS_INFO("So far so good");
    }

    return 0;
}