#ifndef ACO_TSP_TYPES_H
#define ACO_TSP_TYPES_H

#include <vector>

namespace aco
{
    /**
     * Node used by the graph
     */
    struct Node
    {
        Node(double x, double y, int id);

        int id;
        double x{};
        double y{};
        std::vector<std::pair<int, double>> neighbors;

        bool operator==(const Node &other) const;

        /**
         * Get distance between current node and node with node id as node_id
         * @param node_id
         * @return distance
         */
        double get_distance_from_neighbor(int node_id) const;
    };

    struct Graph
    {
    private:
        /**
         * The main graph container (The graph is immutable)
         */
        std::vector<Node> graph_;

        /**
         * Number of nodes in the graph
         */
        int n_nodes_;

        /**
         * Get node pointer from graph corresponding to the node id
         * @param node_id
         * @return
         */
        aco::Node *get_node_from_graph(int node_id);

    public:
        Graph();
        /**
         * Get the number of nodes in a graph
         * @return
         */
        int size() const;

        /**
         * Get the mean of all edge weights in the graph
         * @return
         */
        double mean_edge_weight() const;

        /**
         * Create a new node in Graph with co-ordinates x and y
         * @param x - x coordinate
         * @param y - y coordinate
         * @return
         */
        int create_node_in_graph(double x, double y);

        /**
         * Add edge (onw way) from node_id_from to node_id_to in the graph
         * @param node_id_from
         * @param node_id_to
         */
        void add_edge(int node_id_from, int node_id_to);

        /**
         * Get node from graph corresponding to the node id
         * @param node_id
         * @return
         */
        aco::Node get_node_from_graph(int node_id) const;
    };

} // namespace aco

#endif //ACO_TSP_TYPES_H
