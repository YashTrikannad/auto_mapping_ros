#define DEBUG 1

#include <ros/package.h>

#include "auto_mapping_ros/coverage_planner.h"
#include "auto_mapping_ros/graph_builder.h"
#include "auto_mapping_ros/skeletonizer.h"
#include "auto_mapping_ros//utils.h"

int main()
{
    const auto filepath = ros::package::getPath("auto_mapping_ros") + "/maps/levine.jpg";

    amr::Skeletonizer processor;
    processor.read_map(filepath);
    cv::Mat skeleton = processor.skeletonize();

    cv::Mat map = cv::imread(filepath, 0);

    assert(skeleton.rows == map.rows);
    assert(skeleton.cols == map.cols);

    amr::GraphBuilder builder(skeleton, map);
    builder.build_graph();
    auto graph = builder.get_graph();

    amr::CoveragePlanner planner(&graph);
    planner.compute_sequence();
    std::vector<std::array<int, 2>> sequence = planner.get_sequence();

    std::cout << "size of sequence: " << sequence.size();

    amr::visualize_sequence_on_graph(map, graph, sequence);

    return 0;
}
