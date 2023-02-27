#include <ros/ros.h>
#include <map>
#include <geometry_msgs/PoseArray.h>
#include <interp_cable/sort.hpp>

#include <interp_cable/utilities.hpp>
#include <interp_cable/paraspline.hpp>
#include <interp_cable/matplotlibcpp.h>

namespace plt = matplotlibcpp;
const long fg = plt::figure();

std::vector<std::vector<double>> OpticalReading;
std::vector<double> startN; // Known normal vector for the starting
std::vector<double> startP; // Known starting point coordinates
double theta_max = M_PI;
double theta_min = 0.0;
double dL = 200;
double sigma = 0.25 * dL;

void printv(std::vector<double> &a)
{
    for (int i = 0; i < a.size(); i++)
    {
        std::cout << a[i] << ", ";
    }
    std::cout << std::endl;
}

void NDI_point_callback(const geometry_msgs::PoseArray &p)
{
    // std::cout<<"get in callback\n";
    OpticalReading.clear();
    // startP.clear();
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
        //     startP = temp;
        // }
        // else
        // {
        //     // std::cout<<"length is :"<<i<<std::endl;
        //     OpticalReading.push_back(temp);
        // }
        OpticalReading.push_back(temp);
        temp.clear();
    }

    if (OpticalReading.size() >= 1)
    {
        ROS_INFO("Poses are caught!");
    }
};

void NDI_vector_callback(const geometry_msgs::Pose &v)
{
    startN.clear();
    startN.push_back(v.position.x);
    startN.push_back(v.position.y);
    startN.push_back(v.position.z);
};

void set_startpoint()
{
    startP.clear();
    startP.push_back(-290.720);
    startP.push_back(-382.160);
    startP.push_back(-943.160);
}

void set_direction()
{
    startN.clear();
    startN.push_back(-290.720 - (-364.910));
    startN.push_back(-382.160 - (-418.850));
    startN.push_back(-943.160 - (-907.600));
};

int main(int argc, char **argv)
{
    // Initialize ROS stuff
    //------------------------------------------------------------------------------
    ros::init(argc, argv, "Sorting");
    ros::NodeHandle nh;
    ros::Subscriber NDI_point_sub_;
    ros::Subscriber NDI_vector_sub_;
    ros::Publisher sorted_pub_;

    // get parameters
    //------------------------------------------------------------------------------
    // if(nh.getParam(/theta_max, theta_max)){
    // }

    // Initialize object to plot
    //------------------------------------------------------------------------------
    std::vector<double> plot_x, plot_y, plot_z;

    //------------------------------------------------------------------------------
    // Subscribe to the stray markers
    NDI_point_sub_ = nh.subscribe("/NDI/measured_cp_array", 1, NDI_point_callback);
    // NDI_vector_sub_ = nh.subscribe("/Normal_vec", 10, NDI_vector_callback);
    sorted_pub_ = nh.advertise<geometry_msgs::PoseArray>("/cpp_test", 10);
    set_direction();
    set_startpoint();
    ROS_INFO("%f", startP.size());
    ros::Rate rate(50);

    while (nh.ok())
    {
        // std::cout<<"arrive sub\n";

        //------------------------------------------------------------------------------
        // sort the points here
        if (NDI_point_sub_.getNumPublishers() != 0)
        {
            if (OpticalReading.size() != 0)
            {
                set_startpoint();
                Cable::Sort sort(theta_max, theta_min, dL, sigma, startP, startN);
                sort.fsort(OpticalReading);

                // Publish the result
                geometry_msgs::Pose p_temp;
                geometry_msgs::PoseArray output;
                std::cout << "Optical Reading has " << OpticalReading.size() << " points \n";
                for (int i = 0; i < OpticalReading.size(); i++)
                {
                    p_temp.position.x = OpticalReading[i][0];
                    p_temp.position.y = OpticalReading[i][1];
                    p_temp.position.z = OpticalReading[i][2];

                    output.poses.push_back(p_temp);
                    // std::cout << output.poses[i].position.x << "\n"
                            //   << output.poses[i].position.y << "\n"
                            //   << output.poses[i].position.z << std::endl;
                }
                // std::cout << "output has " << output.poses.size() << " points \n";
                sorted_pub_.publish(output);

                ////////////////////////////////////////////////////////////////////////////
                ///////// Plot Using Matplotlib-cpp Bridge//////////////////////////////////
                ////////////////////////////////////////////////////////////////////////////
                for (int i = 0; i < OpticalReading.size(); i++)
                {
                    plot_x.push_back(OpticalReading[i][0]);
                    plot_y.push_back(OpticalReading[i][1]);
                    plot_z.push_back(OpticalReading[i][2]);
                }

                ////////////////////////////////////////////////////////////////////////////
                ///////////// Perform spline interpolation//////////////////////////////////
                ////////////////////////////////////////////////////////////////////////////
                std::vector<std::vector<double>> PlotSet;
                auto interp_space = linspace(0.0, 1.0, plot_x.size());
                std::vector<double> interp_obj;
                std::vector<double> interp_plot;
                double end_deriv;

                Cable::paraspline *spline;
                for (int i = 0; i < 3; i++)
                {
                    spline = new Cable::paraspline;
                    // ROS_INFO("Good 1");

                    if (i == 0)
                    {
                        interp_obj = plot_x;
                        end_deriv = startN[0];
                    }
                    else if (i == 1)
                    {
                        interp_obj = plot_y;
                        end_deriv = startN[1];
                    }
                    else if (i == 2)
                    {
                        interp_obj = plot_z;
                        end_deriv = startN[2];
                    }
                    // ROS_INFO_STREAM("the length of the interpolation object is " << interp_obj.size());
                    spline->set_boundary(Cable::paraspline::bound_type::first_order, end_deriv,
                                         Cable::paraspline::bound_type::first_order, 0.0);

                    spline->set_points(interp_space, interp_obj, Cable::paraspline::cubic);

                    auto fine_space = linspace(0, 1, 100);

                    for (int j = 0; j < fine_space.size(); j++)
                    {
                        interp_plot.push_back(spline->operator()(fine_space[j]));
                    }

                    PlotSet.push_back(interp_plot);

                    interp_obj.clear();
                    interp_plot.clear();
                    delete spline;
                }

                std::vector<double> temp_x = PlotSet[0];
                std::vector<double> temp_y = PlotSet[1];
                std::vector<double> temp_z = PlotSet[2];

                std::map<std::string, std::string> kwargs;
                kwargs["marker"] = "o";
                kwargs["linestyle"] = "-";
                kwargs["linewidth"] = "1";
                kwargs["markersize"] = "2";
                // plt::plot(temp_x, temp_y); // 2D plot animation works fine
                // plt::plot(plot_x,plot_y,kwargs);

                plt::plot3(temp_x, temp_y, temp_z, kwargs, fg);
                // plt::show();
                plt::pause(0.05);
                plt::clf();
                plot_x.clear();
                plot_y.clear();
                plot_z.clear();
            }
        }
        OpticalReading.clear();
        startP.clear();

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
};