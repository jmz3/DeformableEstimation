#include <ros/ros.h>
#include <map>
#include <eigen3/Eigen/Eigen>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Core>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/TransformStamped.h>
#include <interp_cable/sort.hpp>

#include <interp_cable/utilities.hpp>
#include <interp_cable/paraspline.hpp>
#include <interp_cable/matplotlibcpp.h>

namespace plt = matplotlibcpp;
const long fig = plt::figure();

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

void PointArrayCallback(const geometry_msgs::PoseArray &p)
{
    // std::cout<<"get in callback\n";
    optical_reading.clear();
    // start_point.clear();
    std::vector<double> temp; // Temporary point tuple of (x,y,z)
    //------------------------------------------------------------------------------
    // extract the coordinates form the pose array
    // std::cout<<"arrive here\n";
    for (int i = 0; i < p.poses.size(); i++)
    {
        // std::cout<<"arrive here\n";
        temp.push_back(1000 * p.poses[i].position.x);
        temp.push_back(1000 * p.poses[i].position.y);
        temp.push_back(1000 * p.poses[i].position.z); // times 1000 to transfer the unit from meter back to mm
        // if (i == -1)
        // {
        //     start_point = temp;
        // }
        // else
        // {
        //     // std::cout<<"length is :"<<i<<std::endl;
        //     optical_reading.push_back(temp);
        // }
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

    // Initialize object to plot
    //------------------------------------------------------------------------------
    std::vector<double> plot_x, plot_y, plot_z;

    //------------------------------------------------------------------------------
    // Subscribe to the stray markers
    point_sub = nh.subscribe("/NDI/measured_cp_array", 1, PointArrayCallback);
    direction_sub = nh.subscribe("/NDI/PointerNew/measured_cp", 1, PointerDirectionCallback);
    // NDI_vector_sub_ = nh.subscribe("/Normal_vec", 10, NDI_vector_callback);

    sorted_pub = nh.advertise<geometry_msgs::PoseArray>("/sorted_pts", 10);
    interp_pub = nh.advertise<geometry_msgs::PoseArray>("/interpolated_pts", 10);

    // set the direction of the end point manually
    end_normal.clear();
    end_normal.push_back(0.00067 - (-0.00831));
    end_normal.push_back(-0.01303 - (-0.12501));
    end_normal.push_back(-0.83334 - (-0.84154));

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
            SetStartPoint(); // set the start point to the first point of the cable
            Cable::Sort sort(kThetaMax, kThetaMin, kDeltaLength, kSigma, start_point, start_normal, false);
            sort.SortPoints(optical_reading);

            // Publish the result
            geometry_msgs::Pose pose_temp;

            // std::cout << "Optical Reading has " << optical_reading.size() << " points \n";
            for (int i = 0; i < optical_reading.size(); i++)
            {
                pose_temp.position.x = optical_reading[i][0];
                pose_temp.position.y = optical_reading[i][1];
                pose_temp.position.z = optical_reading[i][2];

                sorted_output.poses.push_back(pose_temp);
                // std::cout << sorted_output.poses[i].position.x << "\n"
                //   << sorted_output.poses[i].position.y << "\n"
                //   << sorted_output.poses[i].position.z << std::endl;
            }
            // std::cout << "sorted_output has " << sorted_output.poses.size() << " points \n";

            ////////////////////////////////////////////////////////////////////////////
            ///////// Plot Using Matplotlib-cpp Bridge//////////////////////////////////
            ////////////////////////////////////////////////////////////////////////////
            for (int i = 0; i < optical_reading.size(); i++)
            {
                plot_x.push_back(optical_reading[i][0]);
                plot_y.push_back(optical_reading[i][1]);
                plot_z.push_back(optical_reading[i][2]);
            }

            ////////////////////////////////////////////////////////////////////////////
            ///////////// Perform spline interpolation /////////////////////////////////
            ////////////////////////////////////////////////////////////////////////////
            std::vector<std::vector<double>> plot_set;
            auto interp_space = Linspace(0.0, 1.0, plot_x.size());
            std::vector<double> interp_obj;
            std::vector<double> interp_plot;
            double start_deriv;
            double end_deriv;

            Cable::paraspline *spline;
            for (int i = 0; i < 3; i++)
            {
                spline = new Cable::paraspline;
                // ROS_INFO("Good 1");

                if (i == 0)
                {
                    interp_obj = plot_x;
                    start_deriv = start_normal[0];
                    end_deriv = end_normal[0];
                }
                else if (i == 1)
                {
                    interp_obj = plot_y;
                    start_deriv = start_normal[1];
                    end_deriv = end_normal[1];
                }
                else if (i == 2)
                {
                    interp_obj = plot_z;
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
                    interp_plot.push_back(spline->operator()(fine_space[j]));
                }

                plot_set.push_back(interp_plot);

                interp_obj.clear();
                interp_plot.clear();
                delete spline;
            }

            std::vector<double> temp_x = plot_set[0];
            std::vector<double> temp_y = plot_set[1];
            std::vector<double> temp_z = plot_set[2];

            std::map<std::string, std::string> args_map;
            args_map["marker"] = "o";
            args_map["linestyle"] = "-";
            args_map["linewidth"] = "1";
            args_map["markersize"] = "2";
            // plt::plot(temp_x, temp_y); // 2D plot animation works fine
            // plt::plot(plot_x,plot_y,kwargs);sorted_output

            plt::plot3(temp_x, temp_y, temp_z, args_map, fig);
            plt::title("Interpolated Cable");
            plt::xlabel("x");
            plt::ylabel("y");
            plt::xlim(-300.0, 600.0);
            plt::ylim(-700.0, 700.0);
            // plt::show();
            plt::pause(0.01);
            plt::clf();
            plot_x.clear();
            plot_y.clear();
            plot_z.clear();

            for (int k = 0; k < plot_set[0].size(); k++)
            {
                pose_temp.position.x = plot_set[0][k];
                pose_temp.position.y = plot_set[1][k];
                pose_temp.position.z = plot_set[2][k];

                interp_output.poses.push_back(pose_temp);
            }

            ROS_INFO_STREAM("The published data has the length " << interp_output.poses.size());
            sorted_pub.publish(sorted_output);
            interp_pub.publish(interp_output);

            sorted_output.poses.clear();
            interp_output.poses.clear();
        }
        else
        {
            ROS_INFO("Marker data is incomplete!");
        }
        optical_reading.clear();
        start_point.clear();

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
};