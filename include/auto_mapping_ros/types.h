//
// Created by yash on 2/19/20.
//

#ifndef AUTO_MAPPING_ROS_TYPES_H
#define AUTO_MAPPING_ROS_TYPES_H

/// Basic Cell of the Graph
struct Node
{
    explicit Node(const std::array<int, 2> &node) : x(node[0]), y(node[1]), mst_key(std::numeric_limits<int>::max())
    {}
    int id;
    int x;
    int y;
    std::vector<Node *> neighbors;
    std::vector<double> neighbors_cost;
    double mst_key;
    std::vector<Node *> mst_child;

    bool operator==(const Node &rhs) const
    {
        return (x == rhs.x) && (y == rhs.y);
    }

    bool operator<(const Node &rhs) const
    {
        return mst_key < rhs.mst_key;
    }
};

struct NodeHasher
{
    std::size_t operator()(const Node& k) const
    {
        using std::size_t;
        using std::hash;

        return ((hash<size_t >()(k.x)^(hash<size_t >()(k.y) << 1)) >> 1);
    }
};

using Graph = std::vector<Node>;

#endif //AUTO_MAPPING_ROS_TYPES_H
