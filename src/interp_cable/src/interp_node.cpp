#include <ros/ros.h>
#include <thread>
#include <eigen3/Eigen/Eigen>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Core>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/TransformStamped.h>
#include <interp_cable/sort.hpp>

#include <interp_cable/utilities.hpp>
#include <interp_cable/paraspline.hpp>

using namespace std::this_thread;

std::vector<std::vector<double>> optical_reading;
std::vector<double> start_normal; // Known normal vector for the starting
std::vector<double> start_point;  // Known starting point coordinates
std::vector<double> end_normal;   // Known normal vector for the ending

geometry_msgs::PoseArray sorted_output;
geometry_msgs::PoseArray interp_output;

int num_of_markers = 0;
double kThetaMax = M_PI;
double kThetaMin = 0.0;
double kDeltaLength = 200;
double kSigma = 0.25 * kDeltaLength;

// callback function to get the NDI point array of stray markers
void PointArrayCallback(const geometry_msgs::PoseArray &p)
{
    optical_reading.clear();
    std::vector<double> temp; // Temporary point tuple of (x,y,z)
    //------------------------------------------------------------------------------
    // extract the coordinates form the pose array

    for (int i = 0; i < p.poses.size(); i++)
    {
        // std::cout<<"arrive here\n";
        // times 1000 to transfer the unit from meter back to mm
        temp.push_back(1000 * p.poses[i].position.x);
        temp.push_back(1000 * p.poses[i].position.y);
        temp.push_back(1000 * p.poses[i].position.z);
        optical_reading.push_back(temp);
        temp.clear();
    }

    if (optical_reading.size() >= 1)
    {
        ROS_INFO("Poses are caught!");
    }
};

void VectorCallback(const geometry_msgs::Pose &v)
{
    start_normal.clear();
    start_normal.push_back(v.position.x);
    start_normal.push_back(v.position.y);
    start_normal.push_back(v.position.z);
};

void PointerDirectionCallback(const geometry_msgs::TransformStamped &pointer)
{
    // Find the direction of the pointer
    // the direction of the cable is the direction of x-axis of the pointer
    end_normal.clear();
    Eigen::Quaterniond quaternion(pointer.transform.rotation.w,
                                  pointer.transform.rotation.x,
                                  pointer.transform.rotation.y,
                                  pointer.transform.rotation.z);

    Eigen::Matrix3d matrix = quaternion.toRotationMatrix();
    for (int i = 0; i < 3; i++)
    {
        // take the first column of the rotation matrix
        // which is the direction vector of x-axis of the pointer frame
        end_normal.push_back(matrix(i, 0));
    }
};

// manually set the start point and direction
void SetStartPoint()
{
    start_point.clear();
    start_point.push_back(104.750);
    start_point.push_back(-86.92);
    start_point.push_back(-719.07);
}

void SetDirection()
{
    start_normal.clear();
    start_normal.push_back(104.75 - (104.64));
    start_normal.push_back(-86.92 - (-158.46));
    start_normal.push_back(-719.07 - (-725.55));
};

int main(int argc, char **argv)
{
    // Initialize ROS stuff
    //------------------------------------------------------------------------------
    ros::init(argc, argv, "Sorting");
    ros::NodeHandle nh;
    ros::Subscriber point_sub;
    ros::Subscriber vector_sub;
    ros::Subscriber direction_sub;
    ros::Publisher sorted_pub;
    ros::Publisher interp_pub;

    // Initialize object for storing the reading
    //------------------------------------------------------------------------------
    std::vector<double> reading_x, reading_y, reading_z;

    //------------------------------------------------------------------------------
    // Subscribe to the stray markers
    point_sub = nh.subscribe("/NDI/measured_cp_array", 1, PointArrayCallback);

    //------------------------------------------------------------------------------
    // Subscribe to the pointer to get the info of the end point
    direction_sub = nh.subscribe("/NDI/PointerNew/measured_cp", 1, PointerDirectionCallback);
    sorted_pub = nh.advertise<geometry_msgs::PoseArray>("/Sorted", 1);
    interp_pub = nh.advertise<geometry_msgs::PoseArray>("/Interp", 1);

    SetDirection();
    SetStartPoint();
    ROS_ASSERT(start_point.size() == 3);

    ros::Rate rate(100);

    while (nh.ok())
    {
        // check for the number of points
        nh.getParam("/marker_num", num_of_markers);
        if (num_of_markers == 0)
        {
            nh.setParam("/ReadingCatched", false);
            ROS_INFO("No markers detected!");
            break;
        }

        //------------------------------------------------------------------------------
        // sort the points here
        if (point_sub.getNumPublishers() != 0 &&
            optical_reading.size() != 0 &&
            optical_reading.size() == num_of_markers)
        // check if the subscriber is connected, and the number of points is correct
        {
            nh.setParam("/ReadingCatched", true);
            SetStartPoint(); // set the start point to the first point of the cable
            Cable::Sort sort(kThetaMax, kThetaMin, kDeltaLength, kSigma, start_point, start_normal, false);
            sort.SortPoints(optical_reading);

            for (int i = 0; i < optical_reading.size(); i++)
            {
                reading_x.push_back(optical_reading[i][0]);
                reading_y.push_back(optical_reading[i][1]);
                reading_z.push_back(optical_reading[i][2]);
            }

            ////////////////////////////////////////////////////////////////////////////
            ///////////// Perform spline interpolation//////////////////////////////////
            ////////////////////////////////////////////////////////////////////////////
            std::vector<std::vector<double>> interp_set;
            std::vector<double> interp_obj;
            std::vector<double> interp_result;
            auto interp_space = Linspace(0.0, 1.0, reading_x.size());
            double start_deriv;
            double end_deriv;

            Cable::paraspline *spline;
            for (int i = 0; i < 3; i++)
            {
                spline = new Cable::paraspline;

                if (i == 0)
                {
                    interp_obj = reading_x;
                    start_deriv = start_normal[0];
                    end_deriv = end_normal[0];
                }
                else if (i == 1)
                {
                    interp_obj = reading_y;
                    start_deriv = start_normal[1];
                    end_deriv = end_normal[1];
                }
                else if (i == 2)
                {
                    interp_obj = reading_z;
                    start_deriv = start_normal[2];
                    end_deriv = end_normal[2];
                }
                // ROS_INFO_STREAM("the length of the interpolation object is " << interp_obj.size());

                spline->set_boundary(Cable::paraspline::bound_type::first_order, start_deriv,
                                     Cable::paraspline::bound_type::first_order, end_deriv);
                spline->set_points(interp_space, interp_obj, Cable::paraspline::cubic);

                auto fine_space = Linspace(0, 1, 100);

                for (int j = 0; j < fine_space.size(); j++)
                {
                    interp_result.push_back(spline->operator()(fine_space[j]));
                }

                interp_set.push_back(interp_result);

                interp_obj.clear();
                interp_result.clear();
                delete spline;
            }

            // Publish the result
            geometry_msgs::Pose pose_temp;

            for (int i = 0; i < optical_reading.size(); i++)
            {
                pose_temp.position.x = optical_reading[i][0];
                pose_temp.position.y = optical_reading[i][1];
                pose_temp.position.z = optical_reading[i][2];

                sorted_output.poses.push_back(pose_temp);
            }

            for (int k = 0; k < interp_set[0].size(); k++)
            {
                pose_temp.position.x = interp_set[0][k];
                pose_temp.position.y = interp_set[1][k];
                pose_temp.position.z = interp_set[2][k];

                interp_output.poses.push_back(pose_temp);
            }

            sorted_pub.publish(sorted_output);
            interp_pub.publish(interp_output);

            reading_x.clear();
            reading_y.clear();
            reading_z.clear();
            sorted_output.poses.clear();
            sleep_for(10ms);
            interp_output.poses.clear();
        }
        else
        {
            nh.setParam("/ReadingCatched", false);
            ROS_INFO("Marker data is incomplete!");
        }
        optical_reading.clear();
        start_point.clear();

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
};