#include <ros/ros.h>

#include "auto_mapping_ros/local_planner.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "auto_mapping_ros_node");
    ros::NodeHandle node_handle;
    ros::spin();
    return 0;
}
