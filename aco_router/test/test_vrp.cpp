#include <iostream>
#include "../vrp_solver.h"

int main()
{
    std::cout << "Basic VRP Problem 9 Nodes: " << std::endl;

    aco::Graph graph;

    // Create nodes in graph
    int id_A = graph.create_node_in_graph(0, 1);
    int id_B = graph.create_node_in_graph(1, 0);
    int id_C = graph.create_node_in_graph(7, 0);
    int id_D = graph.create_node_in_graph(8, 1);
    int id_E = graph.create_node_in_graph(8, 7);
    int id_F = graph.create_node_in_graph(7, 8);
    int id_G = graph.create_node_in_graph(1, 8);
    int id_H = graph.create_node_in_graph(0, 7);
    int id_I = graph.create_node_in_graph(4, 4);

    // Add edges to graph
    graph.add_edge(id_A, id_B);
    graph.add_edge(id_A, id_I);

    graph.add_edge(id_B, id_A);
    graph.add_edge(id_B, id_I);

    graph.add_edge(id_C, id_D);
    graph.add_edge(id_C, id_I);

    graph.add_edge(id_D, id_C);
    graph.add_edge(id_D, id_I);

    graph.add_edge(id_E, id_F);
    graph.add_edge(id_E, id_I);

    graph.add_edge(id_F, id_E);
    graph.add_edge(id_F, id_I);

    graph.add_edge(id_G, id_H);
    graph.add_edge(id_G, id_I);

    graph.add_edge(id_H, id_G);
    graph.add_edge(id_H, id_I);

    graph.add_edge(id_I, id_A);
    graph.add_edge(id_I, id_B);
    graph.add_edge(id_I, id_C);
    graph.add_edge(id_I, id_D);
    graph.add_edge(id_I, id_E);
    graph.add_edge(id_I, id_F);
    graph.add_edge(id_I, id_G);
    graph.add_edge(id_I, id_H);

    // Set Parameters of the Ant Colony Optimization Problem
    aco::IacoParamas params{};
    params.max_iters = 5;
    params.alpha = 1;
    params.beta = 1;
    params.rho = 0.1;
    params.vehicles_available = 4;

    // Solve the TSP using Ant Colony Optimization
    const auto best_route = aco::solve_vrp(graph, params, id_I);
    std::cout << "total fitness value: " << best_route.second << std::endl;
    for(int i=0; i<best_route.first.size(); i++)
    {
        std::cout<< "route " << i << " : ";
        for(const auto& node: best_route.first[i])
        {
            std::cout << node.id << "-";
        }
        std::cout << std::endl;
    }

    std::cout<< "\nGoogle OR Tools Dataset: " << std::endl;
    // https://developers.google.com/optimization/routing/vrp#c++

    aco::Graph google_graph{};

    std::array<int, 17> id;
    id[0] = google_graph.create_node_in_graph(456, 320);
    id[1] = google_graph.create_node_in_graph(228, 0);
    id[2] = google_graph.create_node_in_graph(912, 0);
    id[3] = google_graph.create_node_in_graph(0, 80);
    id[4] = google_graph.create_node_in_graph(114, 80);
    id[5] = google_graph.create_node_in_graph(570, 160);
    id[6] = google_graph.create_node_in_graph(798, 160);
    id[7] = google_graph.create_node_in_graph(342, 240);
    id[8] = google_graph.create_node_in_graph(684, 240);
    id[9] = google_graph.create_node_in_graph(570, 400);
    id[10] = google_graph.create_node_in_graph(912, 400);
    id[11] = google_graph.create_node_in_graph(114, 480);
    id[12] = google_graph.create_node_in_graph(228, 480);
    id[13] = google_graph.create_node_in_graph(342, 560);
    id[14] = google_graph.create_node_in_graph(684, 560);
    id[15] = google_graph.create_node_in_graph(0, 640);
    id[16] = google_graph.create_node_in_graph(798, 640);

    for(int i=0; i<17; i++)
    {
        for(int j=0; j<17; j++)
        {
            if(i!=j)
            {
                google_graph.add_edge(id[i], id[j]);
            }
        }
    }

    params.max_iters = 100;
    params.alpha = 1;
    params.beta = 1;
    params.rho = 0.5;
    params.vehicles_available = 4;
    params.n_ants = -1;
    params.max_route_per_vehicle = -1;

    const auto google_best_route = aco::solve_vrp(google_graph, params, id[0]);
    std::cout << "total fitness value: " << google_best_route.second << std::endl;
    for(int i=0; i<google_best_route.first.size(); i++)
    {
        std::cout<< "route " << i << " : ";
        for(const auto& node: google_best_route.first[i])
        {
            std::cout << node.id << "-";
        }
        std::cout << std::endl;
    }

    return 0;
}
