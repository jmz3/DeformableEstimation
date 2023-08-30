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

    cross[0] = v1[1] * v2[2] - v1[2] * v2[1];
    cross[1] = v1[2] * v2[0] - v1[0] * v2[2];
    cross[2] = v1[0] * v2[1] - v1[1] * v2[0];

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

double Cable::Sort::GetAngle(std::vector<double> &v1, std::vector<double> &v2)
{
    // Compute the angle between two vectors, range from 0 to pi
    std::vector<double> cross_result = Cross(v1, v2);
    double dot = Dot(v1, v2);

    // ROS_INFO("the angle is %f", acos(dot / (Norm(v1) * Norm(v2))));
    return acos(dot / (Norm(v1) * Norm(v2)));
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
    std::vector<double> v_rot(3);

    std::vector<double> cross = Cross(axis, origin);

    // ROS_INFO("Good so far 10");
    for (int i = 0; i < 3; i++)
    {
        v_rot[i] = origin[i] * cos(angle) + cross[i] * sin(angle) + axis[i] * Dot(axis, origin) * (1 - cos(angle));
    }

    return v_rot;
}

void Cable::Sort::Init(std::vector<std::vector<double>> &point_set)
{
    direction_.clear();
    direction_.push_back(start_normal_);

    if (include_start_ == true)
    {
        point_set.insert(point_set.begin(), start_point_);
    }
    else if (include_start_ == false)
    {
        int closest_idx = FindClosestPoint(point_set, start_point_);
        ROS_INFO_STREAM("closest_idx is: " << closest_idx);
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

    int pts_num = point_set.size();

    for (int i = 0; i < pts_num - 2; i++)
    {
        prob_.clear();
        prob_.assign(pts_num, 0.0);

        ROS_INFO_STREAM("first point is: " << point_set[0][0] << ", " << point_set[0][1] << ", " << point_set[0][2]);

        for (int j = i + 1; j < pts_num; j++)
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

            // find the anglular parameter theta
            theta_ = GetAngle(direction_[i], positional_diff_);

            if (theta_ < theta_max_ && theta_ >= theta_min_)
            {
                mean_ = delta_length_ * sin(theta_) / theta_;
                // ROS_INFO_STREAM("theta is: " << theta_);
                // ROS_INFO_STREAM("mean is: " << mean_);
                // ROS_INFO_STREAM("position difference: " << Norm(positional_diff_));
                prob_[j] = (1 / (sigma_ * std::sqrt(2 * M_PI))) * exp(-0.5 * pow(((Norm(positional_diff_) - mean_) / sigma_), 2));
                // ROS_INFO_STREAM("prob is: " << prob_[j]);
            }

            // if (i == 4)
            // {
            //     ROS_INFO_STREAM("node is: " << i);
            //     ROS_INFO_STREAM("point is: " << point_set[j][0] << ", " << point_set[j][1] << ", " << point_set[j][2]);
            //     ROS_INFO_STREAM("theta is: " << theta_);
            //     ROS_INFO_STREAM("mean is: " << mean_);
            //     ROS_INFO_STREAM("position difference: " << Norm(positional_diff_));
            //     ROS_INFO_STREAM("prob is: " << prob_[j]);
            //     ROS_INFO_STREAM("direction is: " << direction_[i][0] << ", " << direction_[i][1] << ", " << direction_[i][2]);
            // }
        }

        // Find the max probability and its corresponding index
        int max_idx = std::max_element(prob_.begin(), prob_.end()) - prob_.begin();

        ROS_INFO_STREAM("max_idx is: " << max_idx);
        ROS_INFO_STREAM("probs are: " << prob_[0] << "," << prob_[1] << "," << prob_[2] << "," << prob_[3] << "," << prob_[4] << "," << prob_[5] << "," << prob_[6] << "," << prob_[7] << "," << prob_[8] << "," << prob_[9]);

        // avoid swapping the points that has already been sorted
        if (max_idx < i + 1)
        {
            // do nothing
        }
        else
        {
            std::swap(point_set[i + 1], point_set[max_idx]);
        }

        // ROS_INFO_STREAM("first point after swap is: " << point_set[0][0] << ", " << point_set[0][1] << ", " << point_set[0][2]);

        positional_diff_.clear();
        for (int k = 0; k < 3; k++)
        {
            // PositionalDiff = (pi+1) - (pi)
            positional_diff_.push_back(point_set[i + 1][k] - point_set[i][k]);
        }

        // Normalize ( Scale to 0 - 1 )
        Scale(positional_diff_, 1.0 / Norm(positional_diff_));
        // ROS_INFO_STREAM("Norm of Normal is"<<fnorm(Normal));
        ROS_ASSERT(Norm(positional_diff_) - 1 < 1e-6); // Ensure the normal is unit vector

        rot_angle_ = M_PI;

        std::vector<double> direction_rot = Rodriguez(direction_[i], positional_diff_, rot_angle_);

        direction_.push_back(direction_rot);

        ROS_INFO_STREAM("the angular increment is: " << GetAngle(direction_rot, positional_diff_) - GetAngle(direction_[i], positional_diff_));
    }
    ROS_INFO_STREAM("point cloud size after sort is: " << point_set.size());
    ROS_INFO("Sort Complete!");
};
