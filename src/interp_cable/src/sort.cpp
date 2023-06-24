#include <ros/ros.h>
#include <functional>
#include <algorithm>
#include <numeric>
#include <cmath>
#include <interp_cable/sort.hpp>
#include <interp_cable/utilities.hpp>

Cable::Sort::Sort(double theta_max,
                  double theta_min,
                  double delta_length,
                  double sigma,
                  std::vector<double> start_point,
                  std::vector<double> start_normal,
                  bool include_start)
    : theta_max_(theta_max), theta_min_(theta_min), delta_length_(delta_length), sigma_(sigma), start_point_(start_point), start_normal_(start_normal), include_start_(include_start)
{
    // Size check
    ROS_ASSERT(start_point.size() == 3);
    ROS_ASSERT(start_normal.size() == 3);

};

Cable::Sort::~Sort(){};

double Cable::Sort::Dot(std::vector<double> &v1, std::vector<double> &v2)
{
    double dot = 0.0;
    for (int i = 0; i < v1.size(); i++)
    {
        dot += v1[i] * v2[i];
    };
    return dot;
};

std::vector<double> Cable::Sort::Cross(std::vector<double> &v1, std::vector<double> &v2)
{
    std::vector<double> cross;
    cross.clear();
    cross.assign(3, 0.0);
    // std::cout << a.size() << ", " << b.size() << std::endl;

    ROS_ASSERT(v1.size() == 3);
    ROS_ASSERT(v2.size() == 3); // Ensure inner product is performed for 3x1 vector.

    // std::cout << "norm of a is: " << fnorm(a) << std::endl;
    // ROS_INFO("A is ");
    //     for (auto x : a)
    // {
    //     std::cout << x << std::endl;
    // }

    cross[0] = v1[1] * v2[2] - v1[2] * v2[1];
    cross[1] = v1[2] * v2[0] - v1[0] * v2[2];
    cross[2] = v1[0] * v2[1] - v1[1] * v2[0];

    // for (auto x : cross)
    // {
    //     std::cout << x << std::endl;
    // }
    return cross;
};

double Cable::Sort::Norm(std::vector<double> &v)
{
    double sum = 0;
    for (int i = 0; i < v.size(); i++)
    {
        sum += v[i] * v[i];
    }
    double norm_result = std::sqrt(sum);

    return norm_result;
}

double Cable::Sort::Angle(std::vector<double> &v1, std::vector<double> &v2)
{
    std::vector<double> cross_result = Cross(v1, v2);
    double norm_result = Norm(cross_result);
    double dot_result = Dot(v1, v2);
    return atan2(norm_result, dot_result);
};

void Cable::Sort::Scale(std::vector<double> &v, double scalar)
{
    for (int i = 0; i < v.size(); i++)
    {
        v[i] = scalar * v[i];
    }
}

std::vector<double> Cable::Sort::Rodriguez(std::vector<double> &origin, std::vector<double> &axis, double angle)
{
    // a is the original vector
    // b is an unit vector that describes the rotation axis
    // angle is the rotation angle along b
    ROS_ASSERT(Norm(axis) < 1 + 1e-6);
    std::vector<double> temp, rot;
    std::vector<std::vector<double>> comp;

    temp.clear();
    // component 1 : v*cos(angle)
    Scale(origin, cos(angle));
    comp.push_back(origin);

    // component 2 : (k x v)*sin(angle)
    temp = Cross(axis, origin);
    Scale(temp, sin(angle));
    comp.push_back(temp);

    // component 3 : k (k dot v)( 1 - cos(angle) )
    Scale(axis, Dot(axis, origin) * (1 - cos(angle)));
    comp.push_back(axis);

    rot.assign(comp[0].size(), 0.0);
    // ROS_INFO("Good so far 10");
    for (int i = 0; i < comp[0].size(); i++)
    {
        rot[i] = comp[0][i] + comp[1][i] + comp[2][i];
    }

    return rot;
}

void Cable::Sort::Init(std::vector<std::vector<double>> &point_set)
{
    direction_.clear();
    direction_.push_back(start_normal_);

    if(include_start_ == true){
        point_set.insert(point_set.begin(), start_point_);
    }
    else if(include_start_ == false){
        int closest_idx = FindClosestPoint(point_set, start_point_);
        point_set.insert(point_set.begin(), 1, point_set[closest_idx]);
        point_set.erase(point_set.begin() + closest_idx + 1);
    }
    
};

void Cable::Sort::SortPoints(std::vector<std::vector<double>> &point_set)
{
    ROS_INFO_STREAM("point cloud size before init is: " << point_set.size());
    Init(point_set);
    ROS_INFO_STREAM("point cloud size after init is: " << point_set.size());
    // i < size-1 because
    // there's no need to find the next point for the last one
    for (int i = 0; i < point_set.size() - 1; i++)
    {
        prob_.clear();
        prob_.assign(point_set.size(), 0.0);

        for (int j = i + 1; j < point_set.size(); j++)
        {
            positional_diff_.clear();

            for (int k = 0; k < 3; k++)
            {
                // PositionalDiff = (pi+1) - (pi)
                positional_diff_.push_back(point_set[j][k] - point_set[i][k]);
            }
            // for (auto x : PositionalDiff)
            // {
            //     std::cout << x << ", ";
            // }
            // std::cout << std::endl;

            // find the angle theta
            theta_ = Angle(direction_[i], positional_diff_);

            if (theta_ < theta_max_ && theta_ >= theta_min_)
            {
                mean_ = 2 * delta_length_ * cos(theta_) / (M_PI - 2 * theta_);
                prob_[j] = (1 / (sigma_ * std::sqrt(2 * M_PI))) * exp(-0.5 * pow(((Norm(positional_diff_) - mean_) / sigma_), 2));
            }
        }

        // Find the max probability and its corresponding index
        int max_idx = std::max_element(prob_.begin(), prob_.end()) - prob_.begin();
        // for (auto x : Prob)
        // {
        //     std::cout << x << ", ";
        // }
        // std::cout << std::endl;
        // std::cout << MaxIdx << std::endl;
        swap(point_set[i + 1], point_set[max_idx]);

        positional_diff_.clear();
        for (int k = 0; k < 3; k++)
        {
            // PositionalDiff = (pi+1) - (pi)
            positional_diff_.push_back(point_set[i + 1][k] - point_set[i][k]);
        }

        normal_ = Cross(direction_[i], positional_diff_);

        // Normalize ( Scale to 0 - 1 )
        Scale(normal_, 1.0 / Norm(normal_));
        // ROS_INFO_STREAM("Norm of Normal is"<<fnorm(Normal));
        ROS_ASSERT(Norm(normal_) - 1 < 1e-6); // Ensure the normal is unit vector

        rot_angle_ = Angle(direction_[i], positional_diff_);

        std::vector<double> direction_rot = Rodriguez(direction_[i], normal_, 2 * rot_angle_);

        direction_.push_back(direction_rot);
    }
    ROS_INFO_STREAM("point cloud size after sort is: " << point_set.size());
    ROS_INFO("Sort Complete!");
};
