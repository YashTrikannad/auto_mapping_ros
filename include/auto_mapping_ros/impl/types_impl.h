#ifndef AUTO_MAPPING_ROS_TYPES_IMPL_H
#define AUTO_MAPPING_ROS_TYPES_IMPL_H

/// Basic Cell of the Graph
Node::Node(const std::array<int, 2> &node, int id) :
        id(id),
        x(node[0]),
        y(node[1]),
        n_lane_intersections(0)
{
}

Node::Node(int x, int y, int id) :
        id(id),
        x(x),
        y(y),
        n_lane_intersections(0)
{
}

bool Node::operator==(const Node &rhs) const
{
    return (x == rhs.x) && (y == rhs.y);
}

std::array<int, 2> Node::get_location() const
{
    return std::array<int, 2>{x, y};
}

/**
 * Creates a deep copy of the input graph
 * @param graph
 * @return
 */
Graph deep_copy_graph(const Graph& graph)
{
    Graph graph_copy{};
    for(const auto& node: graph)
    {
        graph_copy.emplace_back(Node(node.x, node.y, node.id));
    }

    for(int i=0; i<graph.size(); i++)
    {
        std::vector<Node*> neighbors;
        for(const auto& neighbor_node : graph[i].neighbors)
        {
            neighbors.emplace_back([&](int id){
                for(auto& node: graph_copy) if(node.id == id) return &node;
            }(neighbor_node->id));
        }
        graph_copy[i].neighbors = neighbors;
    }

    return graph_copy;
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
