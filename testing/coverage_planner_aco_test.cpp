#define DEBUG 1

#include <ros/package.h>

#include "../aoc_tsp/aco.h"
#include "auto_mapping_ros/coverage_planner.h"
#include "auto_mapping_ros/graph_builder.h"
#include "auto_mapping_ros/skeletonizer.h"

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

    // Convert Graph to aoc::Graph
    const auto aco_graph = amr::convert_to_aoc_graph(graph);

    // Set ACO Params
    aco::AcoParams params{.n_ants=10, .max_iters=100, .alpha=1, .beta=1, .rho=0.05};

    // Find Best Sequence
    const auto sequence_node = aco::solve_tsp(aco_graph, params);

    std::vector<std::array<int, 2>> sequence;
    for(const auto node: sequence_node)
    {
        sequence.emplace_back(std::array<int, 2>{static_cast<int>(node.x), static_cast<int>(node.y)});
    }
    std::cout << "size of sequence: " << sequence.size();

    amr::visualize_sequence_on_graph(map, graph, sequence);

    return 0;
}