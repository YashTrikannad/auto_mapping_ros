#define DEBUG 1

#include <ros/ros.h>
#include <ros/package.h>

#include "auto_mapping_ros/graph_builder.h"
#include "auto_mapping_ros/landmarks.h"
#include "auto_mapping_ros/skeletonizer.h"
#include "auto_mapping_ros/utils.h"

static constexpr int radius_of_interest = 20;

int main()
{
    const auto filepath = ros::package::getPath("auto_mapping_ros")+"/maps/levine.jpg";

    amr::Skeletonizer processor;
    processor.read_map(filepath);
    cv::Mat skeleton = processor.skeletonize();

    cv::Mat map = cv::imread(filepath, 0);

    assert(skeleton.rows == map.rows);
    assert(skeleton.cols == map.cols);

    amr::GraphBuilder builder(skeleton, map);
    builder.build_graph();
    auto graph = builder.get_graph();

    for(const auto& node: graph)
    {
        amr::find_frontiers(node.get_location(), map, radius_of_interest);
    }

    return 0;
}

