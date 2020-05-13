#ifndef AUTO_MAPPING_ROS_TYPES_H
#define AUTO_MAPPING_ROS_TYPES_H

#include <vector>
#include <array>

/// Basic Cell of the Graph
struct Node
{
    explicit Node(const std::array<int, 2> &node, int id);
    explicit Node(int x, int y, int id);

    int id;
    int x;
    int y;

    // Graph Node Neighbor Variables
    std::vector<Node *> neighbors;
    std::vector<double> neighbors_cost;

    // Landmark Information
    int n_lane_intersections;

    bool operator==(const Node &rhs) const;
    std::array<int, 2> get_location() const;
};

using Graph = std::vector<Node>;
Graph deep_copy_graph(const Graph& graph);

struct Plan
{
    std::array<double, 2> start;
    std::array<double, 2> end;
    std::vector<std::array<double, 2>> plan;
};

#include "impl/types_impl.h"

#endif //AUTO_MAPPING_ROS_TYPES_H
