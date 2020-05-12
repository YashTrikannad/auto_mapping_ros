#define DEBUG 0

#include <ros/package.h>

#include "auto_mapping_ros/coverage_planner.h"
#include "auto_mapping_ros/graph_builder.h"
#include "auto_mapping_ros/skeletonizer.h"
#include "auto_mapping_ros/utils.h"

#include "../aco_router/utils.h"
#include "../aco_router/vrp_solver.h"

int main(int argc, char* argv[])
{
    std::vector<std::string> cmd_args{};
    ros::removeROSArgs(argc, argv, cmd_args);
    int n_vehicles = std::stoi(cmd_args.back());
    const auto filepath = ros::package::getPath("auto_mapping_ros") + "/maps/levine.jpg";
    const auto csv_filepath = ros::package::getPath("auto_mapping_ros") + "/csv/sequence";

    amr::Skeletonizer processor;
    processor.read_map(filepath);
    cv::Mat skeleton = processor.skeletonize();

    cv::Mat map = cv::imread(filepath, 0);

    assert(skeleton.rows == map.rows);
    assert(skeleton.cols == map.cols);

    amr::GraphBuilder builder(skeleton, map);
    builder.build_graph();
    auto graph = builder.get_graph();
    auto aco_graph = aco::convert_to_aco_graph(graph);

    // Set ACO Params
    aco::IacoParamas params;
    params.max_iters = 20;
    params.alpha = 1;
    params.beta = 1;
    params.rho = 0.5;
    params.vehicles_available = n_vehicles;
    params.n_ants = -1;
    params.max_route_per_vehicle = -1;

    // Find Best Sequence
    const auto sequences_node = aco::solve_vrp(aco_graph, params);

    for(int i=0; i< sequences_node.first.size(); i++)
    {
        std::vector<std::array<int, 2>> current_sequence{};
        for(const auto& node: sequences_node.first[i])
        {
            current_sequence.emplace_back(std::array<int, 2>{static_cast<int>(node.x), static_cast<int>(node.y)});
        }
        const auto current_csv_filepath = csv_filepath + "_" + std::to_string(i+1) + ".csv";
        amr::write_sequence_to_csv(current_sequence, current_csv_filepath);
    }

    return 0;
}