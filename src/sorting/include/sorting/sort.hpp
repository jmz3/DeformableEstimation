#include <ros/ros.h>
#include <geometry_msgs/PoseArray.h>

namespace Cable
{
    class Sort
    {
    private:
        double theta_max, theta_min, dL;
        double sigma;

        std::vector<double> TempPosition; // Temporary point tuple of (x,y,z)
        std::vector<double> StartP, StartN;
        std::vector<double> Prob; // The probability list for all possible choices

        // There are two vectors: n and dp,
        // n is the normal direction of the previous cable
        // dp is the vector that connects the pi and pi+1
        // the angle between these two vectors can be found by inner product
        std::vector<std::vector<double>> Direction;
        std::vector<double> PositionalDiff;
        std::vector<double> Normal;
        double rot_angle; // Rotational angle between direction vector at pi and direction vector at pi+1
        double theta; // angle between direction vector at pi and difference vector of (pi+1 - pi)
        double mean;

    public:
        Sort(double theta_max, double theta_min, double dL, double sigma,
             std::vector<double> StartP, std::vector<double> StartN);
        ~Sort();

        double fangle(std::vector<double> &a, std::vector<double> &b); // Angle between two vectors
        void init(std::vector<std::vector<double>> &ps);  // Initialize the arguments
        void fsort(std::vector<std::vector<double>> &ps); // sort the input data v based on the sorting algorithm
        double fnorm(std::vector<double> &a); // Norm calculator
        double fdot(std::vector<double> &a, std::vector<double> &b);


        std::vector<double> fscale(std::vector<double> &a, double s);
        std::vector<double> frodriguez(std::vector<double> &origin, std::vector<double> &axis, double angle);
        std::vector<double> fcross(std::vector<double> &a, std::vector<double> &b);
    };
}