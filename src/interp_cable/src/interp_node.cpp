#include <ros/ros.h>
#include <geometry_msgs/PoseArray.h>
#include <std_msgs/Float64MultiArray.h>
#include <interp_cable/sort.hpp>
#include <interp_cable/utilities.hpp>
#include <interp_cable/paraspline.hpp>





std::vector<std::vector<double>> OpticalReading;
std::vector<double> startN; // Known normal vector for the starting
std::vector<double> startP; // Known starting point coordinates
double theta_max = M_PI;
double theta_min = 0.0;
double dL = 200;
double sigma = 0.25 * dL;


void NDI_point_callback(const geometry_msgs::PoseArray &p)
{
    // std::cout<<"get in callback\n";
    OpticalReading.clear();
    startP.clear();
    std::vector<double> temp; // Temporary point tuple of (x,y,z)
    //------------------------------------------------------------------------------
    // extract the coordinates form the pose array
    // std::cout<<"arrive here\n";
    for (int i = 0; i < p.poses.size(); i++)
    {
        // std::cout<<"arrive here\n";
        temp.push_back(p.poses[i].position.x);
        temp.push_back(p.poses[i].position.y);
        temp.push_back(p.poses[i].position.z);
        if (i == 0)
        {
            startP = temp;
        }
        else
        {
            // std::cout<<"length is :"<<i<<std::endl;
            OpticalReading.push_back(temp);
        }
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

void set_direction()
{
    startN.clear();
    startN.push_back(377.1 - 382.0);
    startN.push_back(-1.8 - (-46.1));
    startN.push_back(-1090.6 - (-1070.6));
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
    ros::Publisher plot_pub;

    // get parameters
    //------------------------------------------------------------------------------
    // if(nh.getParam(/theta_max, theta_max)){
    // }

    // Initialize object to plot
    //------------------------------------------------------------------------------
    std::vector<double> plot_x, plot_y, plot_z;

    geometry_msgs::Pose temp_pose;
    geometry_msgs::PoseArray out_sorted;
    std::vector<double> interp_obj;
    std::vector<std::vector<double>> interp_result;
    geometry_msgs::PoseArray out_ir; // Output the fine spaced interpolation result
    

    //------------------------------------------------------------------------------
    // Subscribe to the stray markers
    NDI_point_sub_ = nh.subscribe("/NDI/measured_cp_array", 1, NDI_point_callback);
    NDI_vector_sub_ = nh.subscribe("/Normal_vec", 10, NDI_vector_callback);
    sorted_pub_ = nh.advertise<geometry_msgs::PoseArray>("/cpp_test", 10);
    plot_pub = nh.advertise<geometry_msgs::PoseArray>("plot_array", 10);

    // set_direction();
    ros::Rate rate(50);
    // std::cout<<"arrive sub\n";
    while (nh.ok())
    {
        //------------------------------------------------------------------------------
        // sort the points here
        if (NDI_point_sub_.getNumPublishers() != 0)
        {
            if (OpticalReading.size() != 0)
            {
                Cable::Sort sort(theta_max, theta_min, dL, sigma, startP, startN, false);
                sort.fsort(OpticalReading);

                // Publish the result

                for (int i = 0; i < OpticalReading.size(); i++)
                {
                    temp_pose.position.x = OpticalReading[i][0];
                    temp_pose.position.y = OpticalReading[i][1];
                    temp_pose.position.z = OpticalReading[i][2];

                    out_sorted.poses.push_back(temp_pose);
                    // std::cout << out_sorted.poses[i].position.x << "\n"
                    //           << out_sorted.poses[i].position.y << "\n"
                    //           << out_sorted.poses[i].position.z << std::endl;
                }

                sorted_pub_.publish(out_sorted);

                for (int i = 0; i < OpticalReading.size(); i++)
                {
                    plot_x.push_back(OpticalReading[i][0]);
                    plot_y.push_back(OpticalReading[i][1]);
                    plot_z.push_back(OpticalReading[i][2]);
                }
                auto interp_space = linspace(0.0, 1.0, plot_x.size());
                ////////////////////////////////////////////////////////////////////////////
                ///////////// Perform spline interpolation//////////////////////////////////
                ////////////////////////////////////////////////////////////////////////////

                Cable::paraspline *spline;
                for (int i = 0; i < 3; i++)
                {
                    spline = new Cable::paraspline;

                    spline->set_boundary(Cable::paraspline::bound_type::first_order, 0.0,
                                         Cable::paraspline::bound_type::first_order, 0.0);

                    if (i == 0)
                    {
                        interp_obj = plot_x;
                    }
                    else if (i == 1)
                    {
                        interp_obj = plot_y;
                    }
                    else if (i == 2)
                    {
                        interp_obj = plot_z;
                    }
                    // ROS_INFO_STREAM("the length of the interpolation object is " << interp_obj.size());

                    spline->set_points(interp_space, interp_obj, Cable::paraspline::cubic);

                    auto fine_space = linspace(0, 1, 100);
                    interp_obj.clear();

                    for (int j = 0; j < fine_space.size(); j++)
                    {
                        interp_obj.push_back(spline->operator()(fine_space[j]));
                    }

                    interp_result.push_back(interp_obj);

                    interp_obj.clear();
                    delete spline;
                }

                // ROS_INFO_STREAM("**Plot set has size of "<<interp_result[0].size())

                for (int i = 0; i < interp_result[0].size(); i++)
                {
                    temp_pose.position.x = interp_result[0][i];
                    temp_pose.position.y = interp_result[1][i];
                    temp_pose.position.z = interp_result[2][i];
                    out_ir.poses.push_back(temp_pose);
                }

                plot_pub.publish(out_ir);

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