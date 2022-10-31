#include <ros/ros.h>
#include <geometry_msgs/PoseArray.h>
#include <sorting/sort.hpp>

std::vector<std::vector<double>> PointSet;
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
    PointSet.clear();
    startP.clear();
    ROS_INFO("Poses are caught!");
    std::vector<double> temp; // Temporary point tuple of (x,y,z)
    //------------------------------------------------------------------------------
    // extract the coordinates form the pose array
    for (int i = 0; i < p.poses.size(); i++)
    {
        temp.push_back(p.poses[i].position.x);
        temp.push_back(p.poses[i].position.y);
        temp.push_back(p.poses[i].position.z);
        if (i == 0)
        {
            startP = temp;

        }
        else
        {
            // std::cout<<"arrive here\n";
            PointSet.push_back(temp);
        }
        temp.clear();
    }
};

void NDI_vector_callback(const geometry_msgs::Pose &v)
{
    startN.clear();
    startN.push_back(v.position.x);
    startN.push_back(v.position.y);
    startN.push_back(v.position.z);
};

int main(int argc, char **argv)
{
    // Initialize ROS stuff
    ros::init(argc, argv, "Sorting");
    ros::NodeHandle nh;
    ros::Subscriber NDI_point_sub_;
    ros::Subscriber NDI_vector_sub_;
    ros::Publisher sorted_pub_;

    // get parameters
    // if(nh.getParam(/theta_max, theta_max)){
    // }

    //------------------------------------------------------------------------------
    // Subscribe to the stray markers

    ros::Rate rate(50);

    
    while (nh.ok())
    {

        NDI_point_sub_ = nh.subscribe("/Raw_data", 10, NDI_point_callback);
        NDI_vector_sub_ = nh.subscribe("/Normal_vec", 10, NDI_vector_callback);

        //------------------------------------------------------------------------------
        // sort the points here
        if (NDI_point_sub_.getNumPublishers() != 0)
        {
            if (PointSet.size() != 0)
            {
                Cable::Sort sort(theta_max, theta_min, dL, sigma, startP, startN);
                sort.fsort(PointSet);

                // Publish the result
                geometry_msgs::Pose p_temp;
                geometry_msgs::PoseArray output;
                for (int i = 0; i < PointSet.size(); i++)
                {
                    p_temp.position.x = PointSet[i][0];
                    p_temp.position.y = PointSet[i][1];
                    p_temp.position.z = PointSet[i][2];

                    output.poses.push_back(p_temp);
                    // std::cout << output.poses[i].position.x << "\n"
                    //           << output.poses[i].position.y << "\n"
                    //           << output.poses[i].position.z << std::endl;
                }
                sorted_pub_ = nh.advertise<geometry_msgs::PoseArray>("/cpp_test", 10);
                sorted_pub_.publish(output);

                // // Define the initial point and its orientation
            }
        }
        ros::spinOnce();
    }

    return 0;
};