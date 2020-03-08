#include <ros/ros.h>
#include <ros/package.h>

#include "auto_mapping_ros//skeletonizer.h"

int main()
{
    amr::Skeletonizer processor;
    processor.read_map(ros::package::getPath("auto_mapping_ros")+"/maps/levine.jpg");
    const auto skeleton = processor.skeletonize();
    return 0;
}
