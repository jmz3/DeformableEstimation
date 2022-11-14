#include <ros/ros.h>
#include <functional>
#include <algorithm>
#include <numeric>
#include <cmath>
#include <interp_cable/sort.hpp>

Cable::Sort::Sort(double theta_max, double theta_min, double dL, double sigma,
                  std::vector<double> StartP, std::vector<double> StartN) : theta_max(theta_max), theta_min(theta_min), dL(dL), sigma(sigma), StartP(StartP), StartN(StartN)
{
    // Size check
    ROS_ASSERT(StartP.size() == 3);
    ROS_ASSERT(StartN.size() == 3);
};

Cable::Sort::~Sort(){};

double Cable::Sort::fdot(std::vector<double> &a, std::vector<double> &b)
{
    double dot = 0.0;
    for (int i = 0; i < a.size(); i++)
    {
        dot += a[i] * b[i];
    };
    return dot;
};

std::vector<double> Cable::Sort::fcross(std::vector<double> &a, std::vector<double> &b)
{
    std::vector<double> cross;
    cross.clear();
    cross.assign(3, 0.0);
    // std::cout << a.size() << ", " << b.size() << std::endl;

    ROS_ASSERT(a.size() == 3);
    ROS_ASSERT(b.size() == 3); // Ensure inner product is performed for 3x1 vector.

    // std::cout << "norm of a is: " << fnorm(a) << std::endl;
    // ROS_INFO("A is ");
    //     for (auto x : a)
    // {
    //     std::cout << x << std::endl;
    // }

    cross[0] = a[1] * b[2] - a[2] * b[1];
    cross[1] = a[2] * b[0] - a[0] * b[2];
    cross[2] = a[0] * b[1] - a[1] * b[0];

    // for (auto x : cross)
    // {
    //     std::cout << x << std::endl;
    // }
    return cross;
};

double Cable::Sort::fnorm(std::vector<double> &a)
{
    double sum = 0;
    for (int i = 0; i < a.size(); i++)
    {
        sum += a[i] * a[i];
    }
    double norm_result = std::sqrt(sum);

    return norm_result;
}

double Cable::Sort::fangle(std::vector<double> &a, std::vector<double> &b)
{
    std::vector<double> A = fcross(a, b);
    double B = fnorm(A);
    double C = fdot(a, b);
    return atan2(B, C);
};

void Cable::Sort::fscale(std::vector<double> &a, double scalar)
{
    for (int i = 0; i < a.size(); i++)
    {
        a[i] = scalar * a[i];
    }
}

std::vector<double> Cable::Sort::frodriguez(std::vector<double> &origin, std::vector<double> &axis, double angle)
{
    // a is the original vector
    // b is an unit vector that describes the rotation axis
    // angle is the rotation angle along b
    ROS_ASSERT(fnorm(axis) < 1 + 1e-6);
    std::vector<double> temp, rot;
    std::vector<std::vector<double>> comp;

    temp.clear();
    // component 1 : v*cos(angle)
    fscale(origin, cos(angle));
    comp.push_back(origin);

    // component 2 : (k x v)*sin(angle)
    temp = fcross(axis, origin);
    fscale(temp, sin(angle));
    comp.push_back(temp);

    // component 3 : k (k dot v)( 1 - cos(angle) )
    fscale(axis, fdot(axis, origin) * (1 - cos(angle)));
    comp.push_back(axis);

    rot.assign(comp[0].size(), 0.0);
    // ROS_INFO("Good so far 10");
    for (int i = 0; i < comp[0].size(); i++)
    {
        rot[i] = comp[0][i] + comp[1][i] + comp[2][i];
    }

    return rot;
}

void Cable::Sort::init(std::vector<std::vector<double>> &ps)
{
    Direction.clear();
    Direction.push_back(StartN);

    ps.insert(ps.begin(), StartP);
};

void Cable::Sort::fsort(std::vector<std::vector<double>> &ps)
{
    ROS_INFO_STREAM("point cloud size before init is: " << ps.size());
    init(ps);
    ROS_INFO_STREAM("point cloud size after init is: " << ps.size());
    // i < size-1 because
    // there's no need to find the next point for the last one
    for (int i = 0; i < ps.size() - 1; i++)
    {
        Prob.clear();
        Prob.assign(ps.size(), 0.0);

        for (int j = i + 1; j < ps.size(); j++)
        {
            PositionalDiff.clear();

            for (int k = 0; k < 3; k++)
            {
                // PositionalDiff = (pi+1) - (pi)
                PositionalDiff.push_back(ps[j][k] - ps[i][k]);
            }
            // for (auto x : PositionalDiff)
            // {
            //     std::cout << x << ", ";
            // }
            // std::cout << std::endl;

            // find the angle theta
            theta = fangle(Direction[i], PositionalDiff);

            if (theta < theta_max && theta >= theta_min)
            {
                mean = 2 * dL * cos(theta) / (M_PI - 2 * theta);
                Prob[j] = (1 / (sigma * std::sqrt(2 * M_PI))) * exp(-0.5 * pow(((fnorm(PositionalDiff) - mean) / sigma), 2));
            }
        }

        // Find the max probability and its corresponding index
        int MaxIdx = std::max_element(Prob.begin(), Prob.end()) - Prob.begin();
        // for (auto x : Prob)
        // {
        //     std::cout << x << ", ";
        // }
        // std::cout << std::endl;
        // std::cout << MaxIdx << std::endl;
        swap(ps[i + 1], ps[MaxIdx]);

        PositionalDiff.clear();
        for (int k = 0; k < 3; k++)
        {
            // PositionalDiff = (pi+1) - (pi)
            PositionalDiff.push_back(ps[i + 1][k] - ps[i][k]);
        }

        Normal = fcross(Direction[i], PositionalDiff);

        // Normalize ( Scale to 0 - 1 )
        fscale(Normal, 1.0 / fnorm(Normal));
        // ROS_INFO_STREAM("Norm of Normal is"<<fnorm(Normal));
        ROS_ASSERT(fnorm(Normal) - 1 < 1e-6); // Ensure the normal is unit vector

        rot_angle = fangle(Direction[i], PositionalDiff);

        std::vector<double> direction_rot = frodriguez(Direction[i], Normal, 2 * rot_angle);

        Direction.push_back(direction_rot);
    }
    ROS_INFO_STREAM("point cloud size after sort is: " << ps.size());
    ROS_INFO("Sort Complete!");
};
