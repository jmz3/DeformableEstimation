#include <ros/ros.h>
#include <geometry_msgs/PoseArray.h>

namespace Cable
{
    class Sort
    {
    private:
        double theta_max_, theta_min_, delta_length_;
        double sigma_;
        bool include_start_;

        std::vector<double> temp_position_; // Temporary point tuple of (x,y,z)
        std::vector<double> start_point_, start_normal_;
        std::vector<double> prob_; // The probability list for all possible choices

        // There are two vectors: n and dp,
        // n is the normal direction of the previous cable
        // dp is the vector that connects the pi and pi+1
        // the angle between these two vectors can be found by inner product
        std::vector<std::vector<double>> direction_;
        std::vector<double> positional_diff_;
        std::vector<double> normal_;
        double rot_angle_; // Rotational angle between direction vector at pi and direction vector at pi+1
        double theta_; // angle between direction vector at pi and difference vector of (pi+1 - pi)
        double mean_;

    public:
        Sort(double theta_max, double theta_min, double delta_length, double sigma,
             std::vector<double> start_point, std::vector<double> start_normal, bool include_start);
        ~Sort();

        double GetAngle(std::vector<double> &v1, std::vector<double> &v2); // Angle between two vectors
        void Init(std::vector<std::vector<double>> &point_set);  // Initialize the arguments
        void SortPoints(std::vector<std::vector<double>> &point_set); // sort the input data v based on the sorting algorithm
        double Norm(std::vector<double> &v); // Norm calculator
        double Dot(std::vector<double> &v1, std::vector<double> &v2);


        void Scale(std::vector<double> &v, double scalar);
        std::vector<double> Rodriguez(std::vector<double> &origin, std::vector<double> &axis, double angle);
        std::vector<double> Cross(std::vector<double> &v1, std::vector<double> &v2);
    };
}