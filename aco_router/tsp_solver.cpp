#include <limits>
#include <memory>
#include <random>

#include "utils.h"
#include "tsp_solver.h"

/**
 * Creates a colony of ants (set of improving sub optimal tsp paths)
 * @param colony - colony of ants
 * @param tau - pheromene (desirability obtained till now)
 * @param eta - desirabiliyu of path (inverse of the cost matrix)
 * @param params - ant colony optimization parameters
 */
void create_colony(const aco::Graph& graph,
                   std::vector<std::vector<aco::Node>>& colony,
                   const Eigen::MatrixXd& tau,
                   const Eigen::MatrixXd& eta,
                   const aco::AcoParams& params,
                   const int initial_node_id)
{
    // TODO: Use previous elements from the colony
    colony.clear();

    // Find whether the user wants to set intial node for the ants to start or random
    bool use_random_start = true;
    if(initial_node_id >= 0 && initial_node_id < tau.rows())
    {
        use_random_start = false;
    }

    // Find the initial node for the ant to start
    auto get_intial_node = [&](){
        if(use_random_start)
        {
            std::random_device rd;
            std::mt19937 mt(rd());
            std::uniform_int_distribution<int> int_dist(0, graph.size()-1);
            return graph.get_node_from_graph(int_dist(mt));
        }
        else
        {
            return graph.get_node_from_graph(initial_node_id);
        }
    };

    for(int i=0; i<params.n_ants; i++)
    {
        // Initialize vectors
        std::vector<aco::Node> current_ant_path;
        Eigen::ArrayXd visited = Eigen::ArrayXd::Zero(graph.size());

        // Initial Node
        const auto init_node = get_intial_node();
        visited(init_node.id) = 1;
        current_ant_path.emplace_back(init_node);

        // Move the ant through the n nodes based on the probabilities received in the previous iteration
        for(int j=0; j<graph.size()-1; j++)
        {
            const auto current_node = current_ant_path.back();

            // Calculate Probabilities of next choice of path for the ant
            Eigen::ArrayXd probability_array = Eigen::ArrayXd::Zero(graph.size());
            for(int k=0; k<graph.size(); k++)
            {
                if(visited(k)) continue;
                probability_array(k) = pow(tau(current_node.id, k), params.alpha) * pow(eta(current_node.id, k), params.beta);
            }
            double probability_array_sum = probability_array.sum();
            Eigen::ArrayXd norm_prob_array = probability_array/probability_array_sum;

            // Call Roulette Wheel to get the next node
            int next_node_id = run_roulette_wheel(norm_prob_array);
            visited(next_node_id) = 1;
            const aco::Node next_node = graph.get_node_from_graph(next_node_id);

            // Add next node to the current ant path
            current_ant_path.emplace_back(next_node);
        }

        // Complete the TSP loop
        current_ant_path.emplace_back(init_node);

        // Add current path to the colony
        colony.emplace_back(current_ant_path);
    }
}

/**
 * Updates the pheromone values (tau matrix) based on the paths that the ants moved on and the quality of those paths
 * @param colony - collection of paths that the ants have moved on
 * @param fitness_values - fitness values of each of the paths the ants of the colony moved on
 * @param tau - pheromone matrix
 */
void update_pheromone_value(const std::vector<std::vector<aco::Node>>& colony,
                            const std::vector<double>& fitness_values,
                            Eigen::MatrixXd& tau)
{
    for(int ant_index=0; ant_index < colony.size(); ant_index++)
    {
        for(int node_index=0; node_index < colony[ant_index].size()-1; node_index++)
        {
            const int current_node_id = colony[ant_index][node_index].id;
            const int next_node_id = colony[ant_index][node_index + 1].id;
            tau(current_node_id, next_node_id) = tau(current_node_id, next_node_id) + (1/fitness_values[ant_index]);
        }
    }
}

/**
 * Function to solve the traveling salesman problem for single salesman using ant colony optimization
 * @param graph
 * @param params
 * @return
 */
std::pair<std::vector<aco::Node>, double> aco::solve_tsp(const Graph& graph, const AcoParams& params, int initial_node_id)
{
    // Get Initial Parameters

    // Cost/Distance Matrix
    const Eigen::MatrixXd cost_matrix = get_cost_matrix(graph);

    // Get the initial pheromene matrix
    double tau0 = 10/(graph.size() * graph.mean_edge_weight());
    int n_nodes = graph.size();
    Eigen::MatrixXd tau = Eigen::MatrixXd::Constant(n_nodes, n_nodes, tau0);
    const Eigen::MatrixXd eta = cost_matrix.cwiseInverse();

    // Initialize best route and fitness value
    std::vector<aco::Node> best_route{};
    double best_fitness_value = std::numeric_limits<double>::max();

    std::vector<std::vector<aco::Node>> colony;

    // Main ACO Loop
    for(int i=0; i<params.max_iters; i++)
    {
        // Create Colony
        create_colony(graph, colony, tau, eta, params, initial_node_id);

        // Find All Fitness Values
        std::vector<double> ant_fitness_value(params.n_ants, 0);
        for(int j=0; j<params.n_ants; j++)
        {
            ant_fitness_value[j] = find_fitness_values(cost_matrix, colony.at(j));
        }

        // Find the Queen/ Best Ant Path
        const auto min_index = std::distance(ant_fitness_value.begin(),
                std::min_element(ant_fitness_value.begin(), ant_fitness_value.end()));
        const auto min_value = ant_fitness_value[min_index];
        if(min_value < best_fitness_value)
        {
            best_route = colony[min_index];
            best_fitness_value = min_value;
        }

        // Update pheromone values
        update_pheromone_value(colony, ant_fitness_value, tau);

        // Evaporation
        tau = (1-params.rho)*tau;
    }

    return {best_route, find_fitness_values(cost_matrix, best_route)};
}

