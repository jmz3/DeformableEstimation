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

std::vector<std::vector<double>> OpticalReading;
std::vector<double> startN; // Known normal vector for the starting
std::vector<double> startP; // Known starting point coordinates
std::vector<double> endN;   // Known normal vector for the ending

geometry_msgs::PoseArray sorted_output;
geometry_msgs::PoseArray interp_output;

int num_of_markers = 0;
double theta_max = M_PI;
double theta_min = 0.0;
double dL = 200;
double sigma = 0.25 * dL;

// callback function to get the NDI point array of stray markers
void NDI_point_callback(const geometry_msgs::PoseArray &p)
{
    OpticalReading.clear();
    std::vector<double> temp; // Temporary point tuple of (x,y,z)
    //------------------------------------------------------------------------------
    // extract the coordinates form the pose array

    for (int i = 0; i < p.poses.size(); i++)
    {
        // std::cout<<"arrive here\n";
        temp.push_back(1000 * p.poses[i].position.x);
        temp.push_back(1000 * p.poses[i].position.y);
        temp.push_back(1000 * p.poses[i].position.z); // times 1000 to transfer the unit from meter back to mm
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

void NDI_pointer_callback(const geometry_msgs::TransformStamped &pointer)
{
    // Find the direction of the pointer
    // the direction of the cable is the direction of x-axis of the pointer
    endN.clear();
    Eigen::Quaterniond q(pointer.transform.rotation.w,
                         pointer.transform.rotation.x,
                         pointer.transform.rotation.y,
                         pointer.transform.rotation.z);

    Eigen::Matrix3d m = q.toRotationMatrix();
    for (int i = 0; i < 3; i++)
    {
        // take the first column of the rotation matrix
        // which is the direction vector of x-axis of the pointer frame
        endN.push_back(m(i, 0));
    }
};

// manually set the start point and direction
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
    ros::Subscriber NDI_pointer_sub_;
    ros::Publisher sorted_pub_;
    ros::Publisher interp_pub_;

    // Initialize object for storing the reading
    //------------------------------------------------------------------------------
    std::vector<double> reading_x, reading_y, reading_z;

    //------------------------------------------------------------------------------
    // Subscribe to the stray markers
    NDI_point_sub_ = nh.subscribe("/NDI/measured_cp_array", 1, NDI_point_callback);

    //------------------------------------------------------------------------------
    // Subscribe to the pointer to get the info of the end point
    NDI_pointer_sub_ = nh.subscribe("/NDI/PointerNew/measured_cp", 1, NDI_pointer_callback);
    sorted_pub_ = nh.advertise<geometry_msgs::PoseArray>("/Sorted", 1);
    interp_pub_ = nh.advertise<geometry_msgs::PoseArray>("/Interp", 1);

    set_direction();
    set_startpoint();
    ROS_ASSERT(startP.size() == 3);

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
        if (NDI_point_sub_.getNumPublishers() != 0 &&
            OpticalReading.size() != 0 &&
            OpticalReading.size() == num_of_markers)
        // check if the subscriber is connected, and the number of points is correct
        {
            set_startpoint(); // set the start point to the first point of the cable
            Cable::Sort sort(theta_max, theta_min, dL, sigma, startP, startN, false);
            sort.fsort(OpticalReading);

            for (int i = 0; i < OpticalReading.size(); i++)
            {
                reading_x.push_back(OpticalReading[i][0]);
                reading_y.push_back(OpticalReading[i][1]);
                reading_z.push_back(OpticalReading[i][2]);
            }

            ////////////////////////////////////////////////////////////////////////////
            ///////////// Perform spline interpolation//////////////////////////////////
            ////////////////////////////////////////////////////////////////////////////
            std::vector<std::vector<double>> interp_set;
            std::vector<double> interp_obj;
            std::vector<double> interp_result;
            auto interp_space = linspace(0.0, 1.0, reading_x.size());
            double start_deriv;
            double end_deriv;


            Cable::paraspline *spline;
            for (int i = 0; i < 3; i++)
            {
                spline = new Cable::paraspline;

                if (i == 0)
                {
                    interp_obj = reading_x;
                    start_deriv = startN[0];
                    end_deriv = endN[0];
                }
                else if (i == 1)
                {
                    interp_obj = reading_y;
                    start_deriv = startN[1];
                    end_deriv = endN[1];
                }
                else if (i == 2)
                {
                    interp_obj = reading_z;
                    start_deriv = startN[2];
                    end_deriv = endN[2];
                }
                // ROS_INFO_STREAM("the length of the interpolation object is " << interp_obj.size());

                spline->set_boundary(Cable::paraspline::bound_type::first_order, start_deriv,
                                     Cable::paraspline::bound_type::first_order, end_deriv);
                spline->set_points(interp_space, interp_obj, Cable::paraspline::cubic);

                auto fine_space = linspace(0, 1, 100);

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
            geometry_msgs::Pose p_temp;

            for (int i = 0; i < OpticalReading.size(); i++)
            {
                p_temp.position.x = OpticalReading[i][0];
                p_temp.position.y = OpticalReading[i][1];
                p_temp.position.z = OpticalReading[i][2];

                sorted_output.poses.push_back(p_temp);
            }

            for (int k=0; k<interp_set[0].size(); k++)
            {
                p_temp.position.x = interp_set[0][k];
                p_temp.position.y = interp_set[1][k];
                p_temp.position.z = interp_set[2][k];

                interp_output.poses.push_back(p_temp);
            }

            sorted_pub_.publish(sorted_output);
            interp_pub_.publish(interp_output);


            reading_x.clear();
            reading_y.clear();
            reading_z.clear();
            sorted_output.poses.clear();
            sleep_for(10ms);
            interp_output.poses.clear();
        }
        else
        {
            ROS_INFO("Marker data is incomplete!");
        }
        OpticalReading.clear();
        startP.clear();

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
};