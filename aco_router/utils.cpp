#include <iostream>
#include <random>

#include "utils.h"

/**
 * Get Cost Matrix for the graph required by Ant Colony Optimization
 * @details This function runs the Floyd-Warshall Algorithm for finding shortest path between each pair of the graph
 * @param graph
 * @return
 */
Eigen::MatrixXd get_cost_matrix(const aco::Graph& graph)
{
    const int n_nodes = graph.size();
    Eigen::MatrixXd cost_matrix = Eigen::MatrixXd::Zero(n_nodes, n_nodes);

    for(int i=0; i<n_nodes; i++)
    {
        const auto node_i = graph.get_node_from_graph(i);
        for(int j=0; j<n_nodes; j++)
        {
            if(i != j)
            {
                cost_matrix(i, j) = node_i.get_distance_from_neighbor(j);
            }
        }
    }

    for(int k=0; k<n_nodes; k++)
    {
        for(int i=0; i<n_nodes; i++)
        {
            for(int j=0; j<n_nodes; j++)
            {
                if(i == j) continue;
                if(cost_matrix(i, j) > cost_matrix(i, k) + cost_matrix(k, j))
                {
                    cost_matrix(i, j) = cost_matrix(i, k) + cost_matrix(k, j);
                }
            }
        }
    }

    return cost_matrix;
}

/**
 * Finds fitness value for a single path of ant
 * @param cost_matrix
 * @param ant_path
 * @return
 */
double find_fitness_values(const Eigen::MatrixXd& cost_matrix, const std::vector<aco::Node>& ant_path)
{
    double fitness_value = 0;
    for(int i=0; i<ant_path.size()-1; i++)
    {
        const auto from_node_id = ant_path[i].id;
        const auto to_node_id = ant_path[i+1].id;
        fitness_value += cost_matrix(from_node_id, to_node_id);
    }
    return fitness_value;
}

/**
 * Finds fitness value for a single path of ant
 * @param cost_matrix
 * @param ant_path
 * @return
 */
double find_fitness_values(const Eigen::MatrixXd& cost_matrix, const std::vector<std::vector<aco::Node>>& ant_paths)
{
    double total_fitness_value = 0;
    for(const auto& route: ant_paths)
    {
        total_fitness_value += find_fitness_values(cost_matrix, route);
    }
    return total_fitness_value;
}

/**
 * Find a random index based on probabilities in the probability array
 * @param probability_array
 * @return
 */
int run_roulette_wheel(const Eigen::ArrayXd& probability_array)
{
    Eigen::ArrayXd cumulative_sum = Eigen::ArrayXd::Zero(probability_array.size());
    double current_sum = 0;
    for(int i=0; i<probability_array.size(); i++)
    {
        current_sum += probability_array(i);
        cumulative_sum(i) = current_sum;
    }

    std::random_device rd;
    std::mt19937 mt(rd());
    std::uniform_real_distribution<double> real_dist(0.0, current_sum);

    double rolled_value = real_dist(mt);

    int i=0;
    while(rolled_value > cumulative_sum(i))
    {
        if(i >= cumulative_sum.size())
        {
            std::cout << "invalid: logical error in roulette wheel. \n";
        }
        i++;
    }

    return i;
}
