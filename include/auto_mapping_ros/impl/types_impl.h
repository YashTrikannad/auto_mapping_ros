#ifndef AUTO_MAPPING_ROS_TYPES_IMPL_H
#define AUTO_MAPPING_ROS_TYPES_IMPL_H

/// Basic Cell of the Graph
Node::Node(const std::array<int, 2> &node) :
        x(node[0]),
        y(node[1]),
        mst_key(std::numeric_limits<int>::max()),
        n_lane_intersections(0)
{
}

Node::Node(const std::array<int, 2> &node, int id) :
        id(id),
        x(node[0]),
        y(node[1]),
        mst_key(std::numeric_limits<int>::max()),
        n_lane_intersections(0)
{
}

Node::Node(int x, int y, int id) :
        id(id),
        x(x),
        y(y),
        mst_key(std::numeric_limits<int>::max()),
        n_lane_intersections(0)
{
}

bool Node::operator==(const Node &rhs) const
{
    return (x == rhs.x) && (y == rhs.y);
}

bool Node::operator<(const Node &rhs) const
{
    return mst_key < rhs.mst_key;
}

std::array<int, 2> Node::get_location() const
{
    return std::array<int, 2>{x, y};
}

struct NodeHasher
{
    std::size_t operator()(const Node& k) const
    {
        using std::size_t;
        using std::hash;

        return ((hash<size_t >()(k.x)^(hash<size_t >()(k.y) << 1)) >> 1);
    }
};

#endif //AUTO_MAPPING_ROS_TYPES_IMPL_H
