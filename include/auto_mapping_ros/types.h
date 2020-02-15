#ifndef AUTO_MAPPING_ROS_TYPES_H
#define AUTO_MAPPING_ROS_TYPES_H

namespace amr
{

/// Basic Cell of the Graph
struct Node
{
    explicit Node(const std::array<int, 2> &node) : x(node[0]), y(node[1])
    {}

    int x;
    int y;
    std::vector<Node *> neighbors;
    std::vector<double> neighbors_cost;
};

bool operator==(const Node &lhs, const Node &rhs)
{
    return (lhs.x == rhs.x) && (lhs.y == rhs.y);
}

using Graph = std::vector<Node>;

}

#endif //AUTO_MAPPING_ROS_TYPES_H
