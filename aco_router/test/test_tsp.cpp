#include <iostream>
#include "../tsp_solver.h"

int main()
{
    aco::Graph graph;

    // Create nodes in graph
    int id_A = graph.create_node_in_graph(0, 4);
    int id_B = graph.create_node_in_graph(0, 1);
    int id_C = graph.create_node_in_graph(0, 0);
    int id_D = graph.create_node_in_graph(2, 1);
    int id_E = graph.create_node_in_graph(2, 0);

    // Add edges to graph
    graph.add_edge(id_A, id_B);
    graph.add_edge(id_B, id_A);
    graph.add_edge(id_B, id_C);
    graph.add_edge(id_B, id_D);
    graph.add_edge(id_C, id_B);
    graph.add_edge(id_C, id_E);
    graph.add_edge(id_D, id_B);
    graph.add_edge(id_D, id_E);
    graph.add_edge(id_E, id_C);
    graph.add_edge(id_E, id_D);

    // Set Parameters of the Ant Colony Optimization Problem
    aco::AcoParams params{.n_ants = 2, .max_iters = 3, .alpha=1, .beta=1, .rho=0.05};

    // Solve the TSP using Ant Colony Optimization
    const auto best_route = aco::solve_tsp(graph, params, id_A);
    std::cout << "best fitness value: " << best_route.second << "\n";
    std::cout << "best route: ";
    for(const auto & node: best_route.first)
    {
        std::cout << node.id << "-";
    }

    return 0;
}
