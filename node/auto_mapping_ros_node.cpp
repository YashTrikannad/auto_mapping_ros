#include <ros/ros.h>

#include "auto_mapping_ros/local_planner.h"

static constexpr int n_agents = 1;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "auto_mapping_ros_node");
    ros::NodeHandle nh;
    std::array<amr::LocalPlanner, n_agents> local_planner;
    ros::spin();
    return 0;
}
