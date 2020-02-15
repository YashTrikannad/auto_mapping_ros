#define DEBUG 1

#include "auto_mapping_ros/graph_builder.h"
#include "auto_mapping_ros/skeletonizer.h"

static constexpr auto filepath = "/home/yash/yasht_ws/src/auto_mapping_ros/maps/levine.jpg";

int main()
{
    amr::Skeletonizer processor;
    processor.read_map(filepath);
    cv::Mat skeleton = processor.skeletonize();

    cv::Mat map = cv::imread(filepath, 0);

    assert(skeleton.rows == map.rows);
    assert(skeleton.cols == map.cols);

    amr::GraphBuilder builder(skeleton, map);
    builder.build_graph();
    const auto graph = builder.get_graph();

    return 0;
}
