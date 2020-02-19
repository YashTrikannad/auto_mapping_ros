#include "auto_mapping_ros/coverage_planner.h"

int main()
{
    amr::Graph graph;

    graph.emplace_back(amr::Node(std::array<int, 2>{3, 3}));
    graph.emplace_back(amr::Node(std::array<int, 2>{3, 9}));
    graph.emplace_back(amr::Node(std::array<int, 2>{12, 3}));
    graph.emplace_back(amr::Node(std::array<int, 2>{12, 9}));
    graph[0].neighbors.emplace_back(&graph[1]);
    graph[0].neighbors_cost.emplace_back(6);
    graph[0].neighbors.emplace_back(&graph[2]);
    graph[0].neighbors_cost.emplace_back(7);
    graph[1].neighbors.emplace_back(&graph[0]);
    graph[1].neighbors_cost.emplace_back(6);
    graph[1].neighbors.emplace_back(&graph[3]);
    graph[1].neighbors_cost.emplace_back(9);
    graph[2].neighbors.emplace_back(&graph[0]);
    graph[2].neighbors_cost.emplace_back(7);
    graph[2].neighbors.emplace_back(&graph[3]);
    graph[2].neighbors_cost.emplace_back(5);
    graph[3].neighbors.emplace_back(&graph[1]);
    graph[3].neighbors_cost.emplace_back(9);
    graph[3].neighbors.emplace_back(&graph[2]);
    graph[3].neighbors_cost.emplace_back(5);

    amr::CoveragePlanner planner(&graph);
    planner.compute_sequence();
    std::vector<std::array<int, 2>> sequence = planner.get_sequence();

    std::cout<<"\n\nPLAN";
    for(auto node: sequence)
    {
        std::cout<<"\nx: "<<node[0]<<" y: "<<node[1];
    }

    return 0;
}
