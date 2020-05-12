#include <cmath>
#include <iostream>
#include <limits>

#include "types.h"

aco::Node::Node(double x, double y, int id)
{
    this->id = id;
    this->x = x;
    this->y = y;
}

bool aco::Node::operator==(const aco::Node &other) const
{
    return this->id == other.id;
}

/**
 * Get distance between current node and node with node id as node_id
 * @param node_id
 * @return distance
 */
double aco::Node::get_distance_from_neighbor(int node_id) const
{
    for(const auto& neighbor : this->neighbors)
    {
        if(neighbor.first == node_id)
        {
            return neighbor.second;
        }
    }
    return std::numeric_limits<double>::max();
}

aco::Graph::Graph()
{
    n_nodes_ = 0;
}

int aco::Graph::size() const
{
    return n_nodes_;
}

/**
 * Get the mean of all edge weights in the graph
 * @return
 */
double aco::Graph::mean_edge_weight() const
{
    double sum_edge_weights = 0;
    int n_edges = 0;
    for(const auto& node: graph_)
    {
        for(const auto& neighbor : node.neighbors)
        {
            sum_edge_weights += neighbor.second;
            n_edges++;
        }
    }
    return (sum_edge_weights)/(n_edges);
}

/**
 * Create a new node in Graph with co-ordinates x and y
 * @param x - x coordinate
 * @param y - y coordinate
 * @return
 */
int aco::Graph::create_node_in_graph(double x, double y)
{
    const int id = n_nodes_++;
    this->graph_.emplace_back(Node(x, y, id));
    return id;
}

/**
 * Add edge (onw way) from node_id_from to node_id_to in the graph
 * @param node_id_from
 * @param node_id_to
 */
void aco::Graph::add_edge(int node_id_from, int node_id_to)
{
    const auto node_from = this->get_node_from_graph(node_id_from);
    if(node_from == nullptr)
    {
        std::cout << "Cannot add edge. Node (from) not present in the graph. \n";
        return;
    }
    const auto node_to = this->get_node_from_graph(node_id_to);
    if(node_to == nullptr)
    {
        std::cout << "Cannot add edge. Node (to) not present in the graph. \n";
        return;
    }
    int neighbor_id = node_to->id;
    double distance = sqrt(pow((node_from->x - node_to->x), 2) + pow((node_from->y - node_to->y), 2));
    node_from->neighbors.emplace_back(std::make_pair(neighbor_id, distance));
}

/**
 * Get the node from the graph using node id
 * @param node_id - id of the node
 * @return Node
 */
aco::Node aco::Graph::get_node_from_graph(int node_id) const
{
    return graph_.at(node_id);
}

/**
 * Get pointer to the node from the graph using node id
 * @param node_id - id of the node
 * @return Node*
 */
aco::Node* aco::Graph::get_node_from_graph(int node_id)
{
    return &graph_.at(node_id);
}

namespace std
{
    /**
     * Add hash for aco::Node
     */
    template <>
    struct hash<aco::Node>
    {
        std::size_t operator()(const aco::Node& k) const
        {
            using std::size_t;
            using std::hash;
            using std::string;

            // Compute individual hash values for first,
            // second and third and combine them using XOR
            // and bit shifting:
            return ((hash<int>()(k.id) << 1) >> 1) ^ (hash<int>()(k.id) << 1);
        }
    };
}
