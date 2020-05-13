#define DEBUG 1

#include "auto_mapping_ros/graph_builder.h"
#include "auto_mapping_ros/skeletonizer.h"
#include "../aco_router/utils.h"
#include "../aco_router/vrp_solver.h"

int main(int argc, char* argv[])
{
    int n_vehicles = 1;
    if(argc > 1)
    {
        n_vehicles = std::stoi(argv[1]);
    }
    const auto filepath = "/home/yash/yasht_ws/src/auto_mapping_ros/maps/levine_4.jpg";

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
    const auto aco_graph = aco::convert_to_aco_graph(graph);

    // Set ACO Params
    aco::IacoParamas params{};
    params.max_iters = 1000;
    params.alpha = 1;
    params.beta = 1;
    params.rho = 0.5;
    params.vehicles_available = n_vehicles;
    params.n_ants = -1; /* Default - No of ants is equal to the no. of graph nodes */
    params.max_route_per_vehicle = -1; /* Default - Capacity is calculated based on the no. of vehicles available */

    // Find Best Sequence
    const auto sequence_node = aco::solve_vrp(aco_graph, params);

    for(const auto & i : sequence_node.first)
    {
        std::vector<std::array<int, 2>> sequence;
        for(const auto& node: i)
        {
            sequence.emplace_back(std::array<int, 2>{static_cast<int>(node.x), static_cast<int>(node.y)});
        }
        std::cout << "size of sequence: " << sequence.size() << std::endl;

        amr::visualize_sequence_on_graph(map, graph, sequence);
    }

    return 0;
}